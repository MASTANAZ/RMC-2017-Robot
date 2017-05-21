#pragma once

struct Robot
{
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    int mc1 = 0, mc2 = 0;
    int current_state = -1;
    int control_state = -1;
    bool ls_excv_ext = false;
    bool ls_excv_ret = false;
    bool ls_depo_all = false;
};