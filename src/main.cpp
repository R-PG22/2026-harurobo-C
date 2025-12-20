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

BufferedSerial pc(USBTX, USBRX, 115200);
char c;


// 出力関連の設定
int wheel_pwm[4] = {0};
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
std::array<float, 4> pwm_correction(int tolerance_enc, int target_pwm[4]);

auto clamp = [](auto val, auto min_val, auto max_val){return std::max(std::min(val, max_val), min_val);};

int main(){
    auto state = State::STOP;
    auto prev_state = State::STOP;

    while(true){
        penguin.receive[penguin_number].set(msg.data);
        read_controller();
        if (prev_state == state){
            per_adjust *= 1.5;
            clamp(per_adjust, 0.1f, 1.0f);
        }else{
            per_adjust = 0.1;
        }
        std::array<float, 4> correction_val = pwm_correction(15, state_to_pwm.at(state));

        for (int i = 0; i < 4; ++i){
            penguin.pwm[i] = static_cast<int>(wheel_pwm[i] * per_adjust * correction_val[i] * MAX_PWM);
            clamp(penguin.pwm[i], -MAX_PWM, MAX_PWM);
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

std::array<float, 4> pwm_correction(int tolerance_enc, std::array<int, 4> target_pwm){
    static std::array<float, 4> correction_val = {1.00f, 1.00f, 1.00f, 1.00f};

    for (int i = 0; i < 4; ++i){
        int encoder_val = penguin.receive[i].enc;
        for (int j = 0; j < 4; ++j){
            if (i == j) continue;

            int other_encoder_val = penguin.receive[j].enc;
            if (abs(encoder_val - other_encoder_val) > tolerance_enc){
                if (abs(penguin.pwm[i] - target_pwm[i]) > (MAX_PWM * 0.05f)){
                    correction_val[i] += 0.01f;
                    clamp(correction_val[i], 0.01f, 2.00f);
                }else{
                    correction_val[j] -= 0.01f;
                    clamp(correction_val[j], 0.01f, 2.00f);
                }
            }
        }
    }
    return correction_val;
}