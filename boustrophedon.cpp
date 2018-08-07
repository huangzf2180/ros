#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <iostream>
#include"boustrophedon.h"

using std::cout;
using std::endl;

#define width_pixel 384
#define height_pixel 384

struct Line
{
    int top;
    int bottom;
    int j;
    bool isFree;
};

Area::Area(int id)
{
    area_id = id;
    isComplete = false;
}

vector<Area> get_areaVec()
{
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetMap>("static_map");
    nav_msgs::GetMap map_srv;
    if (!client.call(map_srv))
    {
        ROS_ERROR("Failed to call service");
    }

    nav_msgs::OccupancyGrid map = map_srv.response.map;

    //地图边缘点
    double left = width_pixel, top = height_pixel, right = -1, bottom = -1;
    int line_end[width_pixel] = {0};
    int line_begin[width_pixel];

    for (int i = 0; i < width_pixel; i++)
    {
        line_begin[i] = height_pixel;
    }

    int map_array[height_pixel][width_pixel];
    for (int i = 0; i < map.data.size(); i++)
    {
        map_array[i / width_pixel][i % width_pixel] = map.data[i] + 0;
        if (map_array[i / width_pixel][i % width_pixel] == 0)
        {
            if (left > i % width_pixel)
                left = i % width_pixel;
            if (top > i / width_pixel)
                top = i / width_pixel;
            if (right < i % width_pixel)
                right = i % width_pixel;
            if (bottom < i / width_pixel)
                bottom = i / width_pixel;
            if (line_begin[i % width_pixel] > i / width_pixel)
                line_begin[i % width_pixel] = i / width_pixel;
            if (line_end[i % width_pixel] < i / width_pixel)
                line_end[i % width_pixel] = i / width_pixel;
        }
    }

    vector<Line> div_line;

    int area_count = 0;
    vector<Area> areaVec;
    vector<Line> lineVec;
    for (int j = left; j <= right; j++)
    {
        lineVec.clear();
        int begin = line_begin[j];
        int end = line_end[j];

        //获取单列的连通线段
        bool recordLine = true;
        for (int i = begin; i <= end; i++)
        {
            if (map_array[i][j] != 0 && recordLine)
            {
                Line line;
                line.isFree = true;
                line.top = begin;
                line.bottom = i - 1;
                lineVec.push_back(line);
                recordLine = false;
            }
            if (map_array[i][j] == 0 && !recordLine)
            {
                recordLine = true;
                begin = i;
            }
        }

        //该列最后一条线段
        Line line;
        line.isFree = true;
        line.top = begin;
        line.bottom = end;
        lineVec.push_back(line);


        //逐步构建区域并加入新的区域
        for (vector<Line>::iterator it_line = lineVec.begin(); it_line != lineVec.end(); it_line++)
        {
            for (vector<Area>::iterator it_area = areaVec.begin(); it_area != areaVec.end(); it_area++)
            {
                if ((*it_area).isComplete)
                    continue;
                if (abs((*it_line).top - (*it_area).p_3.x) < 5 && abs((*it_line).bottom - (*it_area).p_4.x) < 5)
                {
                    (*it_area).p_3.x = (*it_line).top;
                    (*it_area).p_3.y = j;
                    (*it_area).p_4.x = (*it_line).bottom;
                    (*it_area).p_4.y = j;
                    (*it_line).isFree = false;
                    break;
                }
            }
            if ((*it_line).isFree)
            {
                area_count++;
                Area area(area_count);
                area.p_1.x = area.p_3.x = (*it_line).top;
                area.p_1.y = area.p_3.y = j;
                area.p_2.x = area.p_4.x = (*it_line).bottom;
                area.p_2.y = area.p_4.y = j;
                areaVec.push_back(area);
                (*it_line).j = j;
                div_line.push_back((*it_line));
            }
        }

        for (vector<Area>::iterator it_area = areaVec.begin(); it_area != areaVec.end(); it_area++)
        {
            if ((*it_area).p_3.y < j)
                (*it_area).isComplete = true;
        }
    }

    //去除长宽过小的区域
    for (vector<Area>::iterator it_area = areaVec.begin(); it_area != areaVec.end();)
    {
        if ((*it_area).p_3.y - (*it_area).p_1.y < 5)
            it_area = areaVec.erase(it_area);
        else
            it_area++;
    }

    for (vector<Area>::iterator it_area = areaVec.begin(); it_area != areaVec.end()-1;it_area++)
    {
        // cout << (*it).top << " " << (*it).bottom << endl;
        for (int i = (*it_area).p_1.x; i <= (*it_area).p_2.x; i++)
        {
            map_array[i][(*it_area).p_1.y] = 50;
        }
        for (int i = (*it_area).p_3.x; i <= (*it_area).p_4.x; i++)
        {
            map_array[i][(*it_area).p_3.y] = 50;
        }
    }

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
    // while (ros::ok())
    // {
    //     map.header.stamp = ros::Time::now();
    //     publisher.publish(map);
    //     loop_rate.sleep();
    // }

    // ROS_INFO("%d", int(areaVec.size()));

    return areaVec;
}