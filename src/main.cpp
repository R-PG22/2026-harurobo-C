#include <iostream>
#include <cmath>
#include "mbed.h"
#include "FirstPenguin.hpp"
#include "kikou.hpp"
#include "controler.hpp"

// 通信系の設定
CAN can(PA_11, PA_12, 1e6);
CANMessage msg1;
constexpr int can_id = 35;
constexpr int penguin_number = 0;
FirstPenguin penguin(can_id, can);

// 出力関連の設定
int16_t pwm1[4] = {0};
bool limit_value[5] = {0};
int16_t encoder_value[5] = {0};
float correction_value[4] = {1.0f, 1.0f, 1.0f, 1.0f};
// constexpr int MAX_PWM = 15000;
// float per_adjust = 0.1; // pwmをゆっくり上げるための係数 0.1f ~ 1.0f
std::string input = "";
std::string last_input = "";

void correction(int16_t (&pwm)[4], int tolerance_value, float p_gain){
    bool each_correction[4][4] = {true};
    if (input != last_input){
        for (int i = 0; i < 4; i++){
            correction_value[i] = 1.0f;
            encoder_value[i] = 0;
        }
    }
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 4; j++){
            if (i == j) each_correction[i][j] = false;
            if (abs(encoder_value[i] - encoder_value[j]) > tolerance_value){
                each_correction[i][j] = false;
            }
        }
    }
    bool is_normal[4] = {true};
    for (int i = 0; i < 4; i++){
        is_normal[i] = each_correction[i][0] || each_correction[i][1] || each_correction[i][2] || each_correction[i][3];
    }
    
    for (int i = 0; i < 4; i++){
        if (!is_normal[i]){
            float adjust_value = 1.0f;
            for (int j = 0; j < 4; j++){
                if (is_normal[j]){
                    adjust_value += (encoder_value[j] - encoder_value[i])*p_gain;
                }
            }
            correction_value[i] = adjust_value;
            if (correction_value[i] < 0.3f) correction_value[i] = 0.3f;
            if (correction_value[i] > 1.8f) correction_value[i] = 1.8f;
        }
        pwm[i] = static_cast<int16_t>(pwm[i] * correction_value[i]);
    }
}

void controller_input(){
    if (controller["u"]) input = "u";
    else if (controller["d"]) input = "d";
    else if (controller["l"]) input = "l";
    else if (controller["r"]) input = "r";
    else input = "";
}

int main(){
    while(true){
        read_controller();
        read_limit(limit_value);
        read_encoder(encoder_value);
        controller_input();

        // コントローラー入力からpwm計算
        pwm_calculation(pwm1[0], controller["L1"], controller["R1"], 15000);  // カード1,l2,r2,pwm_data配列0番
        pwm_calculation(pwm1[1], controller["ci"], controller["sq"], 5000);   // カード2,l,r,pwm_data配列1番
        pwm_calculation(pwm1[2], controller["tri"], controller["cr"], 15000); // カード3,circle,cross,pwm_data配列2番

        // センサー処理
        // sensor_processing(pwm1[0], limit_value[0], 1); // ロジャー下リミット
        // sensor_processing(pwm1[0], limit_value[1], 0); // ロジャー上リミット
        // sensor_processing(pwm1[1], limit_value[2], 1); // garaggaraリミット

        // メカナム計算
        mekanamu_btn(penguin.pwm);
        // 補正処理
        correction(penguin.pwm, 15, 0.001f);

        last_input = input;

        // モタドラへcan送信
        penguin.send();
        CANMessage msg1(2, (const uint8_t *)pwm1, 8);
        can.write(msg1);
    }
}