#include <ros/ros.h>
#include <stdio.h>
#include "lidar_scan/ScanCommand.h"
#include <sys/msg.h>
#include "lidar_scan.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radio_command");
 
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<lidar_scan::ScanCommand>("scan_command");

    sleep(1);

    lidar_scan::ScanCommand srv;
    srv.request.grid_type = 2;
    srv.request.x_rrt = 0;
    srv.request.y_rrt = 0;
    srv.request.x1_ROI_AS  = -1;
    srv.request.y1_ROI_AS  = 0;
    srv.request.x2_ROI_AS  = 2;
    srv.request.y2_ROI_AS  = 3;

    ROS_INFO("Send command to process node");
    if (client.call(srv))
    {
        // display the message
        ROS_INFO("Send data: %d, %d, %d, %d, %d, %d, %d \n", 
            srv.request.grid_type,  srv.request.x_rrt, srv.request.y_rrt, 
            srv.request.x1_ROI_AS, srv.request.y1_ROI_AS, srv.request.x2_ROI_AS, srv.request.y2_ROI_AS);

        ROS_INFO("Response: %d", srv.response.response);
    }
    else
    {
        ROS_ERROR("Failed to call service scan_command");
        return 1;
    }

    return 0;
}