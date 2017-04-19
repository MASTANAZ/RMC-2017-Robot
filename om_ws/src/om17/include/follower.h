// follower.h
// 
// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//

#ifndef _FOLLOWER_H
#define _FOLLOWER_H

#include <vector>

const int PATH_TYPE_STAIGHT = 0;
const int PATH_TYPE_TURN    = 1;

struct Point
{
    float x, y;
};

struct PathSection
{
    Point start, end;
    int type;
};

class Follower
{
public:
    Follower(void);
    ~Follower(void);
    
    void processPath(std::vector<Point> path_points);
private:
    std::vector<PathSection> path_sequence;
};

#endif // _FOLLOWER_H
