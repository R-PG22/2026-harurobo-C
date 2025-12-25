#ifndef CONTROLER_HPP
#define CONTROLER_HPP

#include "mbed.h"
#include <iostream>
#include <map>

int buf_index = 0;
bool stick = false;
char buf[64];
int counter = 0;
std::map<std::string, float> stick_value = {
    {"lx", 0.0},
    {"ly", 0.0},
    {"rx", 0.0},
    {"ry", 0.0},
};
std::map<std::string, bool> controller = {
    {"L1", false},
    {"L2", false},
    {"L3", false},
    {"R1", false},
    {"R2", false},
    {"R3", false},
    {"PS", false},
    {"SH", false},
    {"OP", false},
    {"u", false},
    {"d", false},
    {"l", false},
    {"r", false},
    {"ci", false},
    {"cr", false},
    {"tri", false},
    {"sq", false}};
BufferedSerial pc(USBTX, USBRX, 115200);
std::string last;

void btn_check()
{
    // 判定を行うラムダ（キー文字列を受け取り、on/off を判定して map を更新）
    auto set_if = [&](const std::string &key)
    {
        const std::string on = key + ":p";
        const std::string off = key + ":no_p";
        if (last.rfind(on, 0) == 0)
        { // 先頭一致 (on)
            controller[key] = true;
        }
        else if (last.rfind(off, 0) == 0)
        { // 先頭一致 (off)
            controller[key] = false;
        }
    };

    // 全キーに対してラムダを適用
    for (auto &kv : controller)
    {
        set_if(kv.first);
    }
}
void stick_in()
{
    int read_value = 0;
    if (std::stof(buf) > 0.1 || std::stof(buf) < -0.1)
    {
        int read_value = std::stof(buf);
    }else{
        int read_value = 0;
    }

    if (counter == 0)
    {
        stick_value["lx"] = read_value;
    }
    else if (counter == 1)
    {
        stick_value["ly"] = read_value;
    }
    else if (counter == 2)
    {
        stick_value["rx"] = read_value;
    }
    else if (counter == 3)
    {
        stick_value["ry"] = read_value;
    }
}

// コントローラー読み取り関数
void read_controller()
{
    if (pc.readable())
    {
        char c;
        int num = pc.read(&c, 1);
        if (num > 0)
        {
            if (c == '|' || c == ':')
            {
                last.assign(buf, buf_index);
                // last = std::string(buf);
                if (c == '|')
                {
                    buf_index = 0;
                    if (stick == true)
                    {
                        stick = false;
                        stick_in();
                        counter = 0;
                    }
                    btn_check();
                }
                else
                {
                    if (stick == true)
                    {
                        stick_in();
                        buf_index = 0;
                        counter++;
                    }
                    else if (last == "n")
                    {
                        stick = true;
                        buf_index = 0;
                    }
                    else
                    {
                        buf[buf_index++] = c;
                    }
                }
            }
            else if (buf_index < sizeof(buf) - 1)
            {
                buf[buf_index++] = c;
            }
        }
    }
}

#endif // CONTROLER_HPP