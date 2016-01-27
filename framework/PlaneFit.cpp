#include "PlaneFit.hpp"


#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include <vector>
#include <cstdlib>
#include <fstream>
#include <sstream>


typedef double                      FT;
typedef CGAL::Simple_cartesian<FT>  K;
typedef K::Plane_3                  Plane;
typedef K::Point_3                  Point;




double
detectPlaneQuality(const std::vector<xyz>& corners){

  std::vector<Point> points;
  for(const auto& c : corners){
    points.push_back(Point(c.x, c.y, c.z));
  }

  Plane plane;

  return linear_least_squares_fitting_3(points.begin(),points.end(),plane,CGAL::Dimension_tag<0>());

}
