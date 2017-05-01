#ifndef _TTES_H
#define _TTES_H

namespace ttes
{
    void tick(float dt, int* mc1, int* mc2, float x, float y, float theta);
    bool doSwitch(void);
    void reset(void);
}

#endif /* _TTES_H */