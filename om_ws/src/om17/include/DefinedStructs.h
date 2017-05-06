/**
    DefinedStructs.h

    Blake Nazario-Casey,
    05/02/2017

    This header file defines all the structs used in this test program.
    A similar architecture should be used in the final autonomy module
    on the rover to ensure there are no conflicts.

**/

#pragma once

#include <vector>


/**********************************************************
    CONSTANTS
**********************************************************/

// Constants used to determine path types.
const int PATH_TYPE_HORIZONTAL = 0;
const int PATH_TYPE_VERTICAL = 1;
const int PATH_TYPE_TURN = 2;

/**END CONSTANTS**/


/**********************************************************
    STRUCTS
**********************************************************/

struct Point
{
    Point()
    {
        x = 0;
        y = 0;
    }

    Point(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
    int x, y;
};


// Sectioned paths used by the both the Follower and Main classes.
struct PathSection
{
    Point start, end;
    int type;
};



/**END STRUCTS**/

