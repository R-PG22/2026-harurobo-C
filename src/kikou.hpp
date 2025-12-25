#ifndef KIKOU_HPP
#define KIKOU_HPP

#include "mbed.h"
#include <iostream>
#include "controler.hpp"
CAN can1(PB_12, PB_13, 1e6);
CANMessage msg;
// CANMessage msg1;
// リミットスイッチ読み取り関数
void read_limit(bool (&is_sw_push)[5])
{
    if (can1.read(msg); msg.id == 9)
    {
        uint8_t sw = msg.data[5]; // スイッチの値の内容
        for (int i = 0; i < 5; i++)
        {
            is_sw_push[i] = (sw >> i) & 0x01;
            // シフトで判別する位を最下位ビットへ移動し、1と論理積を行うことで各位の値を判別する。
        }
    }
}

// エンコーダー読み取り関数
void read_encoder(int16_t (&enc)[5])
{
    if (can1.read(msg); msg.id == 9)
    {
        enc[0] = msg.data[7] << 8 | msg.data[6];
    }
    if (can1.read(msg); msg.id == 10)
    {
        for (int i = 0; i < 4; i++)
        {
            enc[i + 1] = msg.data[(i * 2) + 1] << 8 | msg.data[i * 2];
        }
    }
}

// PWM計算関数
void pwm_calculation(int16_t(&pwm_value), bool button1, bool button2, int value)
{
    pwm_value = (button1 - button2) * value;
}

void sensor_processing(int16_t(&pwm_value), bool limit, bool up_down)
{
    if (limit && ((pwm_value > 0) == up_down))
    {
        pwm_value = 0;
    }
}
void mekanamu(int16_t (&pwm)[4])
{
    // メカナム用の足回り計算
    // printf("%d\n",(int)stick_value["lx"]);
    float omuni_value = 15000;
    float power = hypot(stick_value["lx"], stick_value["ly"]);
    float angle = atan2((-1 * stick_value["ly"]), stick_value["lx"]);
    int num2 = (int)(6000.0 * stick_value["rx"]);

    for (int i = 0; i < 4; i++)
    {

        int num = -1 * (int)(sin((M_PI / 180 * (90 * i + 45)) + angle) * power * omuni_value);
        if (num != 0)
        {
            pwm[i] = num;
        }
        else
        {
            pwm[i] = num2;
        }

        if (pwm[i] > 15000)
        {
            pwm[i] = 0;
        }
        else if (pwm[i] < -15000)
        {
            pwm[i] = 0;
        }
    }
}
void mekanamu_btn(int16_t (&pwm)[4])
{
    int omuni_value = 15000;
    if (controller["u"])
    {
        pwm[0] = omuni_value;
        pwm[1] = -omuni_value;
        pwm[2] = -omuni_value;
        pwm[3] = omuni_value;
    }
    else if (controller["d"])
    {
        pwm[0] = -omuni_value;
        pwm[1] = omuni_value;
        pwm[2] = omuni_value;
        pwm[3] = -omuni_value;
    }
    else if (controller["l"])
    {
        pwm[0] = omuni_value;
        pwm[1] = omuni_value;
        pwm[2] = -omuni_value;
        pwm[3] = -omuni_value;
    }
    else if (controller["r"])
    {
        pwm[0] = -omuni_value;
        pwm[1] = -omuni_value;
        pwm[2] = omuni_value;
        pwm[3] = omuni_value;
    }else if (controller["R2"])
    {
        pwm[0] = -omuni_value / 2;
        pwm[1] = -omuni_value / 2;
        pwm[2] = -omuni_value / 2;
        pwm[3] = -omuni_value / 2;
    }else if (controller["L2"])
    {
        pwm[0] = omuni_value / 2;
        pwm[1] = omuni_value / 2;
        pwm[2] = omuni_value / 2;
        pwm[3] = omuni_value / 2;
    }else
    {
        pwm[0] = 0;
        pwm[1] = 0;
        pwm[2] = 0;
        pwm[3] = 0;
    }
}
// センサー処理関数
// limit: リミットスイッチの状態
// up_down: 正負の判定(true:正の場合停止)
// void pid_tick()
// {
//     // pid_control のシグネチャに合わせて呼ぶ（グローバル変数を参照）
//     pid_control(pwm1[2], encoder_value[0], want_k);
// }
float i_control = 0;
int before_error = 0;
float dt = 0.04; // 制御周期
float p_gain = 1.0;
float i_gain = 0.0;
float d_gain = 0.0;
void pid_control(int16_t(&pwm_value), int16_t encoder_value, int16_t want_value)
{
    int error = want_value - encoder_value;
    float p_control = error;
    i_control += error * dt;
    float d_control = (error - before_error) / dt;

    pwm_value = p_gain * p_control + i_gain * i_control + d_gain * d_control;
    if (pwm_value > 10000)
    {
        pwm_value = 10000;
    }
    else if (pwm_value < -10000)
    {
        pwm_value = -10000;
    }
    before_error = error;
    // printf("%d\n", pwm_value);
}

#endif // KIKOU_HPP