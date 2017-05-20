/*
title  : Kalman fusion angle calculation
date   : 08/08/2015
author : houwei
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Kalman.h"

#define AX_ZERO -1181 //加速度计的0偏修正值
#define GX_ZERO -176.85 //陀螺仪的0偏修正

Kalman angle_feng;
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double total_angle = 0;

bool blinkState = false;

void setup() {
    Wire.begin();

    Serial.begin(38400);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}
float Angle = 0.0; //卡尔曼融合最终角度
unsigned long pretime = 0.0;//相当于执行的起始时间
void loop()
{
    double ax_angle = 0.0; //加速度计算得的角度
    double gx_angle = 0.0; //微分的每次角速度算得的角度
    double totgx_angle = 0.0; //总的角速度算得的角度
    unsigned long time = 0; //每执行一次loop所用时间
    unsigned long midtime = 0; //相当于执行的结束时间
    float gyro = 0.0, dt = 0.0;
    if (pretime == 0) pretime = millis();
    midtime = millis();
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax = ax - AX_ZERO;
    /*
    initialize（）中加速的范围设置为2g
    分辨率：16384 LSB/g
    sin(jiaodu) = k(xishu = 0.92) * 3.14 * (jiaodu) / 180
                = （jiasudu） / 16384 (jiasudu/16384 加速度
                                对应范围的实际值）
    */
    ax_angle = ax/263;
    /*
    initialize（）中加速的范围设置为250度/s
    分辨率：131 LSB/S
    gx_angle = ((gy/131)*dt)/1000
    totgx_angle += gx_angle
    */
    gy -= GX_ZERO;
    time = midtime - pretime;

    gyro = gy/131.0;
    gx_angle = gyro * time;
    gx_angle /= 1000.0;
    total_angle += gx_angle;

    dt = time / 1000.0;
    Angle = angle_feng.getAngle(ax_angle, gyro, dt);

    delay(1000);
    Serial.print(ax_angle);Serial.print(", ");
    Serial.print(total_angle); Serial.print(", ");
    Serial.print(Angle);

    pretime = midtime;
}
