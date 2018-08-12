#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <iostream>
#include<ctime>
#include<cstdio>
// #include "boustrophedon.h"
// #include"make_point.cpp"
#include"make_path.cpp"

using std::cout;
using std::endl;



int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "code_test");

    vector<Pixel_Point> pointVec = getPixelPoint();

    

    return 0;
}