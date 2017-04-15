#include "follower.h"

#include <iostream>

Follower::Follower(void)
{
    
}

Follower::~Follower(void)
{
    
}

void Follower::processPath(std::vector<Point> path_points)
{
    std::cout << path_points.size() << std::endl;
    
    int size = path_points.size();

    Point first, last;
    PathSection current_section;

    first = path_points.at(0);

    int i = 0;

    bool turn_section = false;

    while (true)
    {
        Point curr = path_points.at(i);
        Point next = path_points.at(i+1);

        i++;

        // last point reached
        if (i == size - 1)
        {
            last = next;
            
            current_section.start = first;
            current_section.end = last;

            path_sequence.push_back(current_section);
            break;
        }
   
        if (turn_section)
        {
            // straight section
            if (next.y == curr.y)
            {
                turn_section = false;
                
                last = curr;
            
                current_section.start = first;
                current_section.end = last;

                path_sequence.push_back(current_section);

                first = curr;

                continue;
            }
        }
        else
        {
            // turn section starting
            if (next.y != first.y)
            {
                turn_section = true;

                last = curr;

                current_section.start = first;
                current_section.end = last;

                path_sequence.push_back(current_section);

                first = curr;

                continue;
            }
        }
    }

    for (int j = 0; j < path_sequence.size(); ++j)
    {
        std::cout << "start: " << path_sequence.at(j).start.x << ", " << path_sequence.at(j).start.y << std::endl;
        std::cout << "end: " << path_sequence.at(j).end.x << ", " << path_sequence.at(j).end.y << std::endl;
        std::cout << std::endl;
    }
}
