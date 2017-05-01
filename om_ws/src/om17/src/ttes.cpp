#include "ttes.h"

#include <iostream>

//

const int STATE_MOVE = 0;
const int STATE_SURVEY = 1;

//

int ttes_state = STATE_SURVEY;
bool switch_state = false;
float state_timer = 0.0f;
float ty = 1.89f;
int lmc1 = 0;
int lmc2 = 0;
int omc1 = 50;
int omc2 = 50;

//

void ttes::tick(float dt, int* mc1, int* mc2, float x, float y, float theta)
{
    state_timer += dt;

    if (ttes_state == STATE_MOVE)
    {   
        if (state_timer > 5.0f)
        {
            omc1 = lmc1;
            omc2 = lmc2;
            state_timer = 0.0f;
            ttes_state = STATE_SURVEY;
        }
    }
    
    if (ttes_state == STATE_SURVEY)
    {
        lmc1 = 0;
        lmc2 = 0;
        
        if (state_timer > 2.0f)
        {
            float dy = ty - y;
        
            std::cout << "dy: " << dy << std::endl;
            std::cout << "theta: " << theta << std::endl;
            
            // above
            if (dy < 0)
            {
                // looking away
                if (theta > 0)
                {
                    lmc1 = omc1 * 1.05f;
                    lmc2 = omc2 * 0.95f;
                }
                // looking towards
                else if (theta < 0)
                {
                    lmc1 = omc1;
                    lmc2 = omc2;
                }
            }
            // below
            else if (dy > 0)
            {
                // looking towards
                if (theta > 0)
                {
                    lmc1 = omc1;
                    lmc2 = omc2;
                }
                // looking away
                else if (theta < 0)
                {
                    lmc1 = omc1 * 0.8f;
                    lmc2 = omc2 * 1.2f;
                }
            }
        
            std::cout << "mc1: " << lmc1 << std::endl;
            std::cout << "mc2: " << lmc2 << std::endl;
        
            state_timer = 0.0f;
            ttes_state = STATE_MOVE;
        }
    }
    
    if (x > 4.4) switch_state = true;
    
    if (lmc1 > 100) lmc1 = 100;
    if (lmc1 < 0) lmc1 = 0;
    if (lmc2 > 100) lmc2 = 100;
    if (lmc2 < 0) lmc2 = 0;
    
    *mc1 = lmc1;
    *mc2 = lmc2;
}

bool ttes::doSwitch(void)
{
    return switch_state;
}

void ttes::reset(void)
{
    switch_state = false;
}