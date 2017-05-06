#include "follower.h"

#include <iostream>

Follower::Follower(void)
{

}

Follower::~Follower(void)
{

}

/**
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
**/


void Follower::processPath(std::vector<Point> path_points)
{
   std::cout<< "PROCESS PATH IN FOLLOWER CLASS" << std::endl;

/**
   Cases:
            m > 0 && direction is positive == Turn counterclockwise
            m < 0 && direction is positive == Turn clockwise
            m = 0 && direction is positive == move forward
            m = 0 && direction is negative == move backward
            m is undefined == move in direction of travel (up/down/north/south)

        For each case, determine the slope and angle that must be turned to.

        For lengths of path where the slope does not change, remove unnecessary
        points.
    **/

    std::cout << path_points.size() << std::endl;
    int size = path_points.size();

    // Array of slopes : {current slope, next slope}
    //
    // If next slope == current slope, remove point of "next slope"
    // store `next_slope` in a temp variable to become the current slop in the
    // next iteration
    float m[2] = {0.0, 0.0}; // slope of path section

    int indecesToRemove[size] = {0};

    Point tempPoint;
    for (int idx = 0; idx < size; idx++) {
        std::cout<< path_points.at(idx).x << ", " << path_points.at(idx).y << std::endl;

        // Slope at idx -> idx+1
        if (idx != size-1) {
        m[0] = (path_points.at(idx+1).y - path_points.at(idx).y)/(path_points.at(idx+1).x - path_points.at(idx).x);
        std::cout << "Slope for idx -> idx+1"  << ": "<< m[0] << std::endl;
    }
        // Prevent out of bounds error
        if (idx < size-2) {

            // Slope at idx+1 -> idx+2
            if (idx != size-1) {
                m[1] = (path_points.at(idx+2).y - path_points.at(idx+1).y)/(path_points.at(idx+2).x - path_points.at(idx+1).x);
                std::cout << "Slope for idx+1 -> idx+2"  << ": "<< m[1] << std::endl << std::endl;
            }
        }

        // If slope_i = slope_i+1, delete the points in the path_section at i+1
        // and create a temp variable of the point at i+1

        if (m[0] == m[1]) {
            indecesToRemove[idx-1] = 1;
        }

    }

    std::cout<<std::endl<< "*** POINTS TO REMOVE ***" << std::endl;
    int countRemovedPoints = 0;
    for (int idx = size-1; idx >0; idx--) {

        if(indecesToRemove[idx] != 0 && idx < size - 3) {
            tempPoint = path_points.at(idx+2);
            std::cout<< tempPoint.x << ", " << tempPoint.y << std::endl;
            countRemovedPoints += 1;

            path_points.erase(path_points.begin() + idx + 2);
        }
    }

    std::cout<< "POINTS: " <<std::endl;

    int nextSize = path_points.size();
    for (int idx = 0; idx <  nextSize; idx++) {
        std::cout<< path_points.at(idx).x << ", " << path_points.at(idx).y << std::endl;
    }

}
