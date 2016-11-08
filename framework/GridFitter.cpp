#include "GridFitter.hpp"

#include <CGAL/Simple_cartesian.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/linear_least_squares_fitting_2.h>
#include <CGAL/intersections.h>

typedef double                      FT;
typedef CGAL::Simple_cartesian<FT>  K;
//typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef K::Line_2                   Line;
typedef K::Point_2                  Point;
typedef K::Segment_2                Segment;
typedef K::Intersect_2              Intersect;

/*
  1. fit lines through 2D points using CGAL
  http://doc.cgal.org/latest/Principal_component_analysis/Principal_component_analysis_2linear_least_squares_fitting_points_2_8cpp-example.html#a1



  0  1  2   3   4   5   6
  7  8  9  10  11  12  13
  14 15 16 17  18  19  20
  21 22 23 24  25  26  27
  28 29 30 31  32  33  34


  2. calculate crossing points by intersection of lines
  http://doc.cgal.org/latest/Kernel_23/group__intersection__linear__grp.html

*/

std::vector<uv>
gridfit2D(const std::vector<uv>& input){

  // 1. gather points for lines
  std::vector<std::vector<Point> > h_points(CB_HEIGHT);
  std::vector<std::vector<Point> > v_points(CB_WIDTH);


  for(unsigned cp_id = 0; cp_id < CB_WIDTH*CB_HEIGHT; ++cp_id){

    const unsigned h_id = cp_id / CB_WIDTH;
    h_points[h_id].push_back(Point(input[cp_id].u,input[cp_id].v));
    const unsigned v_id = cp_id % CB_WIDTH;
    v_points[v_id].push_back(Point(input[cp_id].u,input[cp_id].v));

  }


  std::vector<Line > h_lines;
  for(const auto& points: h_points){
    Line l;
    const float fitting_quality = linear_least_squares_fitting_2(points.begin(),
								 points.end(),l,CGAL::Dimension_tag<0>());
    h_lines.push_back(l);
  }

  std::vector<Line > v_lines;
  for(const auto& points: v_points){
    Line l;
    const float fitting_quality = linear_least_squares_fitting_2(points.begin(),
								 points.end(),l,CGAL::Dimension_tag<0>());
    v_lines.push_back(l);
  }


  std::vector<uv> output(CB_WIDTH*CB_HEIGHT);
  for(unsigned cp_id = 0; cp_id < CB_WIDTH*CB_HEIGHT; ++cp_id){

    const unsigned h_id = cp_id / CB_WIDTH;
    const unsigned v_id = cp_id % CB_WIDTH;

    CGAL::cpp11::result_of<Intersect(Line, Line)>::type  result = intersection(h_lines[h_id], v_lines[v_id]);
    uv cp = input[cp_id];
    if(result){
      if (const Segment* s = boost::get<Segment>(&*result)) {
	//std::cout << *s << std::endl;
      } else {
	const Point* p = boost::get<Point>(&*result);
	std::cout << "INFO: fitted crossing point id " << cp_id << " to point: " <<  *p << std::endl;
	// This is what we want
	cp.u = p->x();
	cp.v = p->y();
      }
    }

    output[cp_id] = cp; 

  }




  return output;
}

