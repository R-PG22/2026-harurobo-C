#include <map>
#include <array>
#include "mbed.h"
#include "FirstPenguin.hpp"

// 通信系の設定
CAN can(PB_12, PB_13, 1e6);
CANMessage msg;
constexpr int can_id = 35;
constexpr int penguin_number = 0;
FirstPenguin penguin(can_id, can);
// ロリコンは0が横、1が縦とする

BufferedSerial pc(USBTX, USBRX, 115200);
char c;


// 出力関連の設定
array<float, 4> wheel_pwm = {0};
constexpr int MAX_PWM = 15000;
float per_adjust = 0.1; // pwmをゆっくり上げるための係数 0.1f ~ 1.0f

enum class State{
    FORWARD,
    BACK,
    RIGHT,
    LEFT,
    STOP
};
map<char, State> char_to_state = {
    {'w', State::FORWARD},
    {'s', State::BACK},
    {'a', State::LEFT},
    {'d', State::RIGHT},
    {'x', State::STOP}
};
map<State, std::array<int, 4>> state_to_pwm = {
    {State::FORWARD, {1, 1, -1, -1}},
    {State::BACK,    {-1, -1, 1, 1}},
    {State::RIGHT,   {-1, 1, 1, -1}},
    {State::LEFT,    {1, -1, -1, 1}},
    {State::STOP,    {0, 0, 0, 0}}
};

State state;

int per_encoder = 1024;
int prev_encoder[4] = {0};

void read_controller();
void pwm_correction(int tolerance_enc);

auto clamp = [](auto val, auto min_val, auto max_val){return std::max(std::min(val, max_val), min_val);};

int main(){
    auto state = State::STOP;
    auto prev_state = State::STOP;

    while(true){
        penguin.receive[penguin_number].set(msg.data);
        read_controller();
        if (prev_state == state){
            per_adjust *= 1.5f;
            clamp(per_adjust, 0.1f, 1.0f);
        }else{
            per_adjust = 0.1f;
        }
        wheel_pwm = pwm_correction(15, 0.1f);

        for (int i = 0; i < 4; ++i){
            penguin.pwm[i] = static_cast<int>(wheel_pwm[i] * per_adjust * MAX_PWM);
            clamp(int(penguin.pwm[i]), -MAX_PWM, MAX_PWM);
        }
        penguin.send();
        prev_state = state;
    }
}

void read_controller(){
    int len = pc.read(&c, 1);
    if (len > 0){
        if (c == 'w'){
            state = State::FORWARD;
        }else if (c == 's'){
            state = State::BACK;
        }else if (c == 'a'){
            state = State::LEFT;
        }else if (c == 'd'){
            state = State::RIGHT;
        }else if (c == 'x'){
            state = State::STOP;
        }
    }
    for (int i = 0; i < 4; ++i){
        wheel_pwm[i] = state_to_pwm[state][i];
    }
}

array<float, 4> pwm_correction(int tolerance_enc, float p_gain){
    array<float, 4> correction_pwm = {penguin.pwm[0], penguin.pwm[1], penguin.pwm[2], penguin.pwm[3]};
    int diff_01 = penguin.receive[0].enc - penguin.receive[1].enc;
    int diff_12 = penguin.receive[1].enc - penguin.receive[2].enc;
    int diff_23 = penguin.receive[2].enc - penguin.receive[3].enc;
    int diff_30 = penguin.receive[3].enc - penguin.receive[0].enc;
    switch(state){
        case State::FORWARD:
        case State::BACK:
            if (abs(penguin.receive[0].enc) > tolerance_enc){
                if (abs(diff_01) > tolerance_enc){
                    correction_pwm[0] -= diff_01 * p_gain;
                    correction_pwm[1] += diff_01 * p_gain;
                }
                if (abs(diff_23) > tolerance_enc){
                    correction_pwm[2] -= diff_23 * p_gain;
                    correction_pwm[3] += diff_23 * p_gain;
                }
                int diff_01_23 = (penguin.receive[0].enc + penguin.receive[1].enc) / 2 - 
                                (penguin.receive[2].enc + penguin.receive[3].enc) / 2;
                if (abs(diff_01_23) > tolerance_enc){
                    correction_pwm[0] -= diff_01_23 * p_gain;
                    correction_pwm[1] -= diff_01_23 * p_gain;
                    correction_pwm[2] += diff_01_23 * p_gain;
                    correction_pwm[3] += diff_01_23 * p_gain;
                }
            }
            break;
        case State::RIGHT:
        case State::LEFT:
            if (abs(penguin.receive[1].enc) > tolerance_enc){
                if (abs(diff_12) > tolerance_enc){
                    correction_pwm[1] -= diff_12 * p_gain;
                    correction_pwm[2] += diff_12 * p_gain;
                }
                if (abs(diff_30) > tolerance_enc){
                    correction_pwm[2] -= diff_30 * p_gain;
                    correction_pwm[3] += diff_30 * p_gain;
                }
                int diff_12_30 = (penguin.receive[1].enc + penguin.receive[2].enc) / 2 - 
                                (penguin.receive[3].enc + penguin.receive[0].enc) / 2;
                if (abs(diff_12_30) > tolerance_enc){
                    correction_pwm[0] -= diff_12_30 * p_gain;
                    correction_pwm[1] -= diff_12_30 * p_gain;
                    correction_pwm[2] += diff_12_30 * p_gain;
                    correction_pwm[3] += diff_12_30 * p_gain;
                }
            }
            break;
        case State::STOP:
            for (int i = 0; i < 4; ++i){
                penguin.receive[i].enc = 0;
            }
            break;
        default:
            break;
    }
    return correction_pwm;
}