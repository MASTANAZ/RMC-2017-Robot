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

////////////////////////////////////////////////////////////////////////////////
// NODE VARIABLES
////////////////////////////////////////////////////////////////////////////////

char** grid = nullptr;

ros::Publisher world_pub;
ros::Subscriber phobos_world_sub;
ros::Subscriber deimos_world_sub;

////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
////////////////////////////////////////////////////////////////////////////////

void init(void);

// Publisher for World messages.
void worldPublisher();

// Callback for subscriber messages
void chatterCallback(const std_msgs::String::ConstPtr& msg);

void cleanup(void);

////////////////////////////////////////////////////////////////////////////////
// ENTRY POINT
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    ros::init(argc, argv, "world");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(3);

    init();

    chatter_pub = node_handle.advertise<std_msgs::String>("world_update", 1000);
    phobos_world_sub = node_handle.subscribe("phobos_world_update", 1000, chatterCallback);
    deimos_world_sub = node_handle.subscribe("deimos_world_update", 1000, chatterCallback);

    

    while (ros::ok())
    {
        ros::spinOnce();
        worldPublisher();

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
            grid[y][x] = '0';    
            
        }
    }
}


/* World Publisher
 * Publishes an aggregated world vector
 *
 * Blake Nazario-Casey
*/
void worldPublisher()
{
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world";

    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);
}


/* World Publisher callback
 * Prints the subscriber message that it reads based on the world_update 
 * publisher defined above.
 *
 * @param msg The message received from the publisher.
 *
 * Blake Nazario-Casey
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void cleanup(void)
{
    for (unsigned i = 0; i < GRID_HEIGHT; ++i)
    {
        delete [] grid[i];
    }
    delete [] grid;
}

