// CREATED BY HARRIS NEWSTEDER & BLAKE NAZARIO-CASEY
//
// DESCRIPTION:
//   
//   World Map ROS node that subscribes to each rover's map updates, converts to
//   a single vector, and publishes both to each rover. Each rover subscribes to
//   and recieves the updated map with each rover's position information, and 
//   information about the environment.


////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////

#include <cstdlib>                           // standard C library functions
#include <vector>                            // vector formatting
#include <sstream>                           // strings

#include "ros/ros.h"                         // ROS library
#include "geometry_msgs/Pose2D.h"            // For OpenCV positioning data
#include "std_msgs/String.h"                 // For ROS string-type messages
#include "std_msgs/MultiArrayLayout.h"       // For formatting array-types
#include "std_msgs/MultiArrayDimension.h"    // For formatting array-types
#include "std_msgs/Int32MultiArray.h"        // For ROS vector-type messages

#include "om17/CellCost.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
////////////////////////////////////////////////////////////////////////////////

const char ID_OBSTACLE    = '*';
const char ID_PHOBOS      = 'P';
const char ID_DEIMOS      = 'D';
const char ID_PATH_PHOBOS = '-';
const char ID_PATH_DEIMOS = '+';

const unsigned int GRID_WIDTH  = 24;
const unsigned int GRID_HEIGHT = 12;

const float TILE_SIZE = 7.38f / (float)GRID_WIDTH;

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

char** grid = nullptr;

ros::Subscriber phobos_pose_sub;
ros::Subscriber deimos_pose_sub;
ros::Subscriber world_cost_sub;

// phobos grid position
unsigned phobos_x = 0;
unsigned phobos_y = 0;

// deimmos grid position
unsigned deimos_x = 0;
unsigned deimos_y = 0;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(void);
void tick(void);
void cleanup(void);

void phobosPoseCallback(const geometry_msgs::Pose2D& pose);
void deimosPoseCallback(const geometry_msgs::Pose2D& pose);
void worldCostCallback(const om17::CellCost& cell_cost);

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    // ROS initialization
    ros::init(argc, argv, "world");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(3);

    init();

    phobos_pose_sub = node_handle.subscribe("/phobos/pose", 1, phobosPoseCallback);
    deimos_pose_sub = node_handle.subscribe("/deimos/pose", 1, deimosPoseCallback);
    world_cost_sub = node_handle.subscribe("/world_cost", 1, worldCostCallback)

    while (ros::ok())
    {
        ros::spinOnce();
        
        tick();
        
        loop_rate.sleep();    
    }

    cleanup();

    return EXIT_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DEFINITIONS
////////////////////////////////////////////////////////////////////////////////

void init(void)
{
    grid = new char*[GRID_HEIGHT];

    for (unsigned i = 0; i < GRID_HEIGHT; ++i)
    {
        grid[i] = new char[GRID_WIDTH];
    }

    // initialize grid values here
    for (unsigned y = 0; y < GRID_HEIGHT; ++y)
    {
        for (unsigned x = 0; x < GRID_WIDTH; ++x)
        {
            grid[y][x] = '.';
        }
    }
}

void tick(void)
{
    for (unsigned y = 0; y < GRID_HEIGHT; ++y)
    {
        for (unsigned x = 0; x < GRID_WIDTH; ++x)
        {
            grid[y][x] = '.';
        }
    }
    
    grid[deimos_y][deimos_x] = ID_DEIMOS;
    grid[phobos_y][phobos_x] = ID_PHOBOS;
    
    for (unsigned y = 0; y < GRID_HEIGHT; ++y)
    {
        for (unsigned x = 0; x < GRID_WIDTH; ++x)
        {
            std::cout << grid[y][x] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void cleanup(void)
{
    for (unsigned i = 0; i < GRID_HEIGHT; ++i)
    {
        delete [] grid[i];
    }
    delete [] grid;
}

void phobosPoseCallback(const geometry_msgs::Pose2D& pose)
{
    phobos_x = (unsigned)(pose.x / TILE_SIZE);
    phobos_y = (unsigned)(pose.y / TILE_SIZE);
}

void deimosPoseCallback(const geometry_msgs::Pose2D& pose)
{
    deimos_x = (unsigned)(pose.x / TILE_SIZE);
    deimos_y = (unsigned)(pose.y / TILE_SIZE);
}

void worldCostCallback(const om17::CellCost& cell_cost)
{

}
