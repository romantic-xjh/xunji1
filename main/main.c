/*
 * ESP32-S3 循迹 + 超声波避障小车 (main.c: 状态机与循迹决策)
 *
 *  - 4 路红外循迹: 高电平 = 黑线  (sensor.c)
 *  - HC-SR04 超声波测距           (sensor.c)
 *  - 双电机 PWM 开环调速          (motor.c)
 *
 * 状态机:
 *  - 循迹态: 距离 >= 10cm, 按红外状态巡线
 *  - 避障态: 距离 < 10cm 时无论在不在黑线上都进入, 一律向右转,
 *            若障碍一直在就一直右转, 直到重新碰到黑线再回循迹态
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "sensor.h"

/* 避障距离阈值 */
#define OBSTACLE_DIST_CM    10.0f

/* 控制周期 */
#define CTRL_PERIOD_MS      10

/* ========================= 状态机 ========================= */
typedef enum {
    MODE_TRACK = 0,     /* 循迹 */
    MODE_AVOID  = 1,    /* 避障(右转找线) */
} car_mode_t;

static car_mode_t s_mode = MODE_TRACK;

/* ========================= 权重式循迹参数 ========================= */
#define STEER_GAIN     110      /* 权重 -> 占空比换算系数 */
#define STEER_MAX      4       /* 偏差限幅, 决定最大转弯力度 */
#define DUTY_MIN       80      /* 内侧轮最小占空比, 防止最边边时被钳到 0 停转 */

/*
 * 权重表: IR1~IR4 从左到右的位置权重
 * 负 = 线偏左(需左转), 正 = 线偏右(需右转)
 * 离中间越远权重越大 -> 偏离越远转弯力度越大
 * 想改成别的力度组合只改这张表即可
 */
static const int8_t ir_weight[4] = {-3, -2, 2, 3};   /* IR1 最左 ~ IR4 最右 */

/* ================================================================== */
/*                             循迹逻辑                                */
/* ================================================================== */

/*
 * 权重式循迹:
 *  1. 把压线的传感器权重累加成偏差 error (线偏左 error<0, 偏右 error>0)
 *  2. 按比例换算成左右轮差速: 左轮 = 基础速度 + error*系数, 右轮相反
 *
 * 力度举例 (权重 {-3,-1,1,3}, 基础速度 550, 系数 60):
 *  0100 中左压线  error=-1  左轮 490 右轮 610  轻微左转
 *  1100 左侧两个  error=-3  左轮 370 右轮 730  大力左转
 *  1000 只有最左  error=-3  左轮 370 右轮 730  大力左转(线已偏出很远)
 *  0010/0001/0011 同理向右
 */
static void track_control(void)
{
    static int last_error = 0;      /* 全丢线时沿用上次的偏差去找线 */
    int s = ir_read_all();

    /* 全压线(直角/宽线): 直行; 全丢线的情况在下面用 last_error 处理 */
    if (s == 0b1111) {
        car_forward();
        return;
    }
    if (s == 0b0000) {
        car_forward();
        return;
    }
    /* 加权求偏差 */
    int error = 0;
    if (s & 0x8) error += ir_weight[0];     /* IR1 最左 */
    if (s & 0x4) error += ir_weight[1];     /* IR2 中左 */
    if (s & 0x2) error += ir_weight[2];     /* IR3 中右 */
    if (s & 0x1) error += ir_weight[3];     /* IR4 最右 */

    /* 偏差限幅 (如 1100 累加得 -4, 限到 -3) */
    if (error > STEER_MAX) error = STEER_MAX;
    if (error < -STEER_MAX) error = -STEER_MAX;
    if (s != 0) last_error = error;         /* 有效压线时记忆偏差 */
    else        error = last_error;         /* 丢线时按记忆偏差继续找 */

    /* 差速: error<0 线在左 -> 左轮减慢右轮加快 -> 左转 */
    int duty_l = SPEED_BASE + error * STEER_GAIN;
    int duty_r = SPEED_BASE - error * STEER_GAIN;

    /* 占空比限幅: 内侧轮保留最小速度, 防止最边边(error=±3/±4)被钳到 0 停转 */
    if (duty_l < DUTY_MIN) duty_l = DUTY_MIN;
    if (duty_r < DUTY_MIN) duty_r = DUTY_MIN;
    if (duty_l > 1023) duty_l = 1023;
    if (duty_r > 1023) duty_r = 1023;

    car_diff(duty_l, duty_r);
}

/* ================================================================== */
/*                             主任务                                  */
/* ================================================================== */

static void car_task(void *arg)
{
    float dist;

    while (1) {
        /* 每个控制周期测一次距离 */
        dist = ultrasonic_get_distance();

        if (s_mode == MODE_AVOID) {
            /* 避障状态: 一律向右转, 直到重新碰到黑线 */
            car_turn_right();

            /* 中间两个传感器任意一个压到黑线 -> 回到循迹 */
            int s = ir_read_all();
            if (s & 0b0110) {
                s_mode = MODE_TRACK;
                printf("avoid -> track\n");
            } else if (dist < OBSTACLE_DIST_CM) {
                printf("avoiding, dist=%.1fcm, keep turning right\n", dist);
            }
        } else {
            /* 循迹状态: 距离 < 10cm 时无论在不在黑线上都进入避障 */
            if (dist < OBSTACLE_DIST_CM) {
                s_mode = MODE_AVOID;
                printf("obstacle %.1fcm -> avoid mode\n", dist);
            } else {
                track_control();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}

void app_main(void)
{
    motor_init();
    sensor_init();
    car_stop();
    printf("car ready, start tracking\n");

    xTaskCreate(car_task, "car_task", 4096, NULL, 5, NULL);
}
