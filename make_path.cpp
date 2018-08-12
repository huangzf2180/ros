#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <iostream>
#include <vector>

using std::cout;
using std::endl;
using std::vector;

#define width_pixel 384
#define height_pixel 384
#define SCAN_RADIUS 0.3
#define RESOLUTION 0.05

struct Pixel_Point
{
    int x;
    int y;
};

void setPixel(int map_array[][width_pixel], int start_x, int start_y, int orientation, int scan_radius_pixel)
{

    int x, y;

    switch (orientation)
    {
    case 1:
        x = -1;
        y = 0;
        break;
    case 2:
        x = 0;
        y = 1;
        break;
    case 3:
        x = 1;
        y = 0;
        break;
    case 4:
        x = 0;
        y = -1;
        break;
    default:
        cout << "orientation error!" << endl;
        break;
    }

    while (scan_radius_pixel--)
    {
        start_x += x;
        start_y += y;
        if (map_array[start_x][start_y] == 0)
            map_array[start_x][start_y] = -1;
    }
}

vector<Pixel_Point> getPixelPoint()
{

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap map_srv;
    while (!client.call(map_srv))
    {
        ROS_ERROR("Failed to call service");
        ros::Duration(5).sleep();
    }

    nav_msgs::OccupancyGrid map = map_srv.response.map;

    //地图边缘点
    int left = width_pixel, top = height_pixel, right = -1, bottom = -1;

    //地图像素点数组
    int map_array[height_pixel][width_pixel];

    for (int i = 0; i < map.data.size(); i++)
    {
        map_array[i / width_pixel][i % width_pixel] = map.data[i] + 0;
        if (map_array[i / width_pixel][i % width_pixel] == 100)
        {
            cout << i / width_pixel << " " << i % width_pixel << endl;
            if (left > i % width_pixel)
                left = i % width_pixel;
            if (top > i / width_pixel)
                top = i / width_pixel;
            if (right < i % width_pixel)
                right = i % width_pixel;
            if (bottom < i / width_pixel)
                bottom = i / width_pixel;
        }
    }

    int scan_radius_pixel = SCAN_RADIUS / RESOLUTION + 0.5;
    cout << top << bottom << left << right << endl;

    //数组越界未处理
    for (int i = top; i <= bottom; i++)
    {
        for (int j = left; j <= right; j++)
        {
            cout << i << " " << j << endl;
            if (map_array[i][j] == 100)
            {
                if (map_array[i - 1][j] != 100)
                {
                    setPixel(map_array, i, j, 1, scan_radius_pixel);
                }
                if (map_array[i][j + 1] != 100)
                {
                    setPixel(map_array, i, j, 2, scan_radius_pixel);
                }
                if (map_array[i + 1][j] != 100)
                {
                    setPixel(map_array, i, j, 3, scan_radius_pixel);
                }
                if (map_array[i][j - 1] != 100)
                {
                    setPixel(map_array, i, j, 4, scan_radius_pixel);
                }
            }
        }
    }

    // cout << 666 << endl;

    ros::Publisher publisher = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
    ros::Rate loop_rate(10);
    for (int i = 0; i < height_pixel; i++)
    {
        for (int j = 0; j < width_pixel; j++)
        {
            map.data[i * width_pixel + j] = map_array[i][j];
        }
    }

    // ROS_INFO("%d", int(areaVec.size()));
    while (ros::ok())
    {
        map.header.stamp = ros::Time::now();
        publisher.publish(map);
        loop_rate.sleep();
    }
}