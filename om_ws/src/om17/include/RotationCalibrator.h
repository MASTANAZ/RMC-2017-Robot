// RotationCalibrator.h
//
// CREATED BY BLAKE NAZARIO-CASEY
//

#pragma once


#include <vector>
#include "DefinedStructs.h"


class RotationCalibrator
{
public:
    RotationCalibrator(void);
    ~RotationCalibrator(void);

    void calibrateForRotation();

    float timeForDegreesRotation;

private:
    static const float degreesToRotate = 30.0f;
};

