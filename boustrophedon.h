#ifndef BOURSTROPHEDON_H
#define BOURSTROPHEDON_H

#include <vector>

using std::vector;

class Area
{

public:
  struct Point
  {
    int x;
    int y;
  };
  int area_id;
  bool isComplete;
  Point p_1, p_2, p_3, p_4;

  Area(int);

private:
};

vector<Area> get_areaVec();

#endif