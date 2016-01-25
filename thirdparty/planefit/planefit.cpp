// Example program for the linear_least_square_fitting function
// on a set of 3D triangles
#include <CGAL/Simple_cartesian.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <vector>

#include <cstdlib>
#include <fstream>

typedef double                      FT;
typedef CGAL::Simple_cartesian<FT>  K;
typedef K::Plane_3                  Plane;
typedef K::Point_3                  Point;
int main(int argc, char** argv)
{
#if 0
 193.982 203.198 145.567 213.253 203.661 146.133 232.199 203.852 146.341 251.155 204.092 146.69 270.11 204.695 146.656 288.964 204.847 147.143 307.871 205.29 147.135 193.585 222.414 145.733 212.982 222.524 145.764 231.765 222.817 145.968 250.989 223.335 146.247 269.838 223.436 147.051 288.692 223.843 146.953 307.549 224.149 147.307 193.286 241.416 145.629 212.421 241.731 146.191 231.592 241.846 146.209 250.484 242.119 146.304 269.536 242.517 147.063 288.335 242.741 147.16 307.257 242.987 147.418 192.95 260.653 145.937 212.182 260.754 145.946 231.227 260.993 146.255 250.306 261.261 146.604 269.182 261.395 146.786 288.081 261.681 147.134 306.89 261.954 146.918 192.463 279.801 145.632 211.708 279.955 146.002 230.865 280.151 146.203 249.899 280.279 146.389 268.891 280.538 147.039 287.778 280.623 147.152 306.709 280.905 147.341

#endif

  if(argc == 1){
    //system("/home/steppo/Desktop/my-git/rgbd-calib/thirdparty/planefit/planefit 0 0 0 1 0 0 1 1 0 0 1 0");
    system("/home/steppo/Desktop/my-git/rgbd-calib/thirdparty/planefit/planefit 193.982 203.198 145.567 213.253 203.661 146.133 232.199 203.852 146.341 251.155 204.092 146.69 270.11 204.695 141.656 288.964 204.847 147.143 307.871 205.29 147.135 193.585 222.414 145.733 212.982 222.524 145.764 231.765 222.817 145.968 250.989 223.335 146.247 269.838 223.436 147.051 288.692 223.843 146.953 307.549 224.149 147.307 193.286 241.416 145.629 212.421 241.731 146.191 231.592 241.846 146.209 250.484 242.119 146.304 269.536 242.517 147.063 288.335 242.741 147.16 307.257 242.987 147.418 192.95 260.653 145.937 212.182 260.754 145.946 231.227 260.993 146.255 250.306 261.261 146.604 269.182 261.395 146.786 288.081 261.681 147.134 306.89 261.954 146.918 192.463 279.801 145.632 211.708 279.955 146.002 230.865 280.151 146.203 249.899 280.279 146.389 268.891 280.538 147.039 287.778 280.623 147.152 306.709 280.905 147.341");

    float quality;
    std::ifstream qualityfile("/tmp/planequality");
    qualityfile >> quality;
    qualityfile.close();
    std::cout << "quality is: " << quality  << std::endl;
    return 0;
  }

  std::vector<Point> points;

  float x,y,z;
  unsigned xyz = 0;
  for(int i = 1; i != argc; ++i){
    ++xyz;
    if(xyz == 1){
      x = atof(argv[i]);
    }
    else if(xyz == 2){
      y = atof(argv[i]);
    }
    else{ // xyz == 3
      z = atof(argv[i]);
      points.push_back(Point(x,y,z));
      //std::cout << "point: " << x << "," << y << "," << z << std::endl;
      xyz = 0;
    }

  }
  //std::cout << "number of points: " << points.size() << std::endl;


  Plane plane;
  // fit plane to whole triangles
  FT quality = linear_least_squares_fitting_3(points.begin(),points.end(),plane,CGAL::Dimension_tag<0>());
  //std::cout << "internal computed quality: " << quality << std::endl;
  //std::cout << "internal computed normal: " << plane << std::endl;
  std::ofstream qualityfile("/tmp/planequality");
  qualityfile << quality;
  qualityfile.close();
  return 0;
}


