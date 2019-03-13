#include "LPCNearestNeighbourSearch.hpp"



#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
//#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/property_map.h>
#include <boost/iterator/zip_iterator.hpp>
#include <utility>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     Point_3;
typedef boost::tuple<Point_3,int>                           Point_and_int;
//typedef CGAL::Random_points_in_cube_3<Point_3>              Random_points_iterator;
typedef CGAL::Search_traits_3<Kernel>                       Traits_base;
typedef CGAL::Search_traits_adapter<Point_and_int,
  CGAL::Nth_of_tuple_property_map<0, Point_and_int>,
  Traits_base>                                              Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits>          K_neighbor_search;
typedef K_neighbor_search::Tree                             Tree;
typedef K_neighbor_search::Distance                         Distance;


  
LPCNearestNeighbourSearch::LPCNearestNeighbourSearch(const std::vector<surfel>* surfels)
  : m_surfels(surfels),
    m_tree()
{
  std::vector<Point_3>  points;
  std::vector<size_t>   indices;
  size_t idx = 0;
  for(const auto& s : *m_surfels){
    points.push_back(Point_3(s.p[0], s.p[1], s.p[2]));
    indices.push_back(idx);
    ++idx;
  }

  m_tree = new Tree(
		    boost::make_zip_iterator(boost::make_tuple( points.begin(),indices.begin() )),
		    boost::make_zip_iterator(boost::make_tuple( points.end(),indices.end() ) )  
		    );
}
  
LPCNearestNeighbourSearch::~LPCNearestNeighbourSearch(){
  delete reinterpret_cast<Tree*>(m_tree);
}

std::vector<surfel>
LPCNearestNeighbourSearch::search(const surfel& s, unsigned num_neighbours) const{
  std::vector<surfel> result;
  
  Point_3 query(s.p[0], s.p[1], s.p[2]);
  K_neighbor_search search(*(reinterpret_cast<Tree*>(m_tree)), query, num_neighbours);
  
  for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
    result.push_back((*m_surfels)[boost::get<1>(it->first)]);
  }
  
  return result;
}

std::vector<size_t>
LPCNearestNeighbourSearch::search_id(const surfel& s, unsigned num_neighbours) const{
  std::vector<size_t> result;
  
  Point_3 query(s.p[0], s.p[1], s.p[2]);
  K_neighbor_search search(*(reinterpret_cast<Tree*>(m_tree)), query, num_neighbours);
  
  for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
    result.push_back(boost::get<1>(it->first));
  }
  
  return result;
}


