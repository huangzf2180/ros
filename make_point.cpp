#include"boustrophedon.h"

struct Point{
    double x;
    double y;
};

vector<Point> make_point(){

    vector<Point> pointVec;
    vector<Area> areaVec = get_areaVec();

    for(vector<Area>::iterator it = areaVec.begin(); it != areaVec.end(); it++){
        int pixel_pos_x = ((*it).p_1.x + (*it).p_2.x) / 2;
        int pixel_pos_y = ((*it).p_1.y + (*it).p_3.y) / 2;
        double x = (pixel_pos_y - 200) * 0.05;
        double y = (pixel_pos_x - 200) * 0.05;
        Point point;
        point.x = x;
        point.y = y;
        pointVec.push_back(point);
        std::cout<<(*it).p_1.x<<" "<<(*it).p_2.x<<std::endl;
        std::cout<<x<<" "<<y<<std::endl;
    }

    return pointVec;
}