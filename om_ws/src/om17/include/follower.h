// follower.h
//
// CREATED BY HARRIS NEWSTEDER AND BLAKE NAZARIO-CASEY
//

#ifndef _FOLLOWER_H
#define _FOLLOWER_H

#include <vector>
#include "DefinedStructs.h"

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
