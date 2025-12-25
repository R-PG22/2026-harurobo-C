#include <iostream>
#include <cmath>
#include "mbed.h"
#include "FirstPenguin.hpp"
#include "kikou.hpp"
#include "controler.hpp"

// 通信系の設定
CAN can(PB_12, PB_13, 1e6);
CANMessage msg1;
constexpr int can_id = 35;
constexpr int penguin_number = 0;
FirstPenguin penguin(can_id, can);
// ロリコンは0が横、1が縦とする


// 出力関連の設定
int16_t pwm1[4] = {0};
bool limit_value[5] = {0};
int16_t encoder_value[5] = {0};
// constexpr int MAX_PWM = 15000;
// float per_adjust = 0.1; // pwmをゆっくり上げるための係数 0.1f ~ 1.0f

int main(){
    while(true){
        read_controller();
        read_limit(limit_value);
        read_encoder(encoder_value);

        // コントローラー入力からpwm計算
        pwm_calculation(pwm1[0], controller["L1"], controller["R1"], 15000);  // カード1,l2,r2,pwm_data配列0番
        pwm_calculation(pwm1[1], controller["ci"], controller["sq"], 5000);   // カード2,l,r,pwm_data配列1番
        pwm_calculation(pwm1[2], controller["tri"], controller["cr"], 15000); // カード3,circle,cross,pwm_data配列2番

        // センサー処理
        // sensor_processing(pwm1[0], limit_value[0], 1); // ロジャー下リミット
        // sensor_processing(pwm1[0], limit_value[1], 0); // ロジャー上リミット
        // sensor_processing(pwm1[1], limit_value[2], 1); // garaggaraリミット

        // メカナム計算
        mekanamu(penguin.pwm);

        // モタドラへcan送信
        penguin.send();
        CANMessage msg1(2, (const uint8_t *)pwm1, 8);
        can.write(msg1);
    }
}