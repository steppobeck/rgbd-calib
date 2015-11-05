#include "NearestNeighbourSearch.hpp"

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



  
  NearestNeighbourSearch::NearestNeighbourSearch(const std::vector<nniSample>& spoints)
    : shared_spoints(spoints),
      m_tree()
  {
    

    std::vector<Point_3>  points;
    std::vector<size_t>   indices;
    size_t idx = 0;
    for(const auto& sp : shared_spoints){
      points.push_back(Point_3(sp.s_pos.x, sp.s_pos.y, sp.s_pos.z));
      indices.push_back(idx);
      ++idx;
    }
    

    m_tree = new Tree(
		      boost::make_zip_iterator(boost::make_tuple( points.begin(),indices.begin() )),
		      boost::make_zip_iterator(boost::make_tuple( points.end(),indices.end() ) )  
		      );

    
  }
  
  NearestNeighbourSearch::~NearestNeighbourSearch()
  {}
  

  std::vector<nniSample>
  NearestNeighbourSearch::search(const nniSample& ipolant,unsigned num_neighbours) const{
    std::vector<nniSample> result;

    Point_3 query(ipolant.s_pos.x, ipolant.s_pos.y, ipolant.s_pos.z);
    K_neighbor_search search(*(reinterpret_cast<Tree*>(m_tree)), query, num_neighbours);

    for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
      result.push_back(shared_spoints[boost::get<1>(it->first)]);
    }


    return result;
#if 0    
    const unsigned int K = 5;
    Point_3 query(0.0, 0.0, 0.0);
    Distance tr_dist;
    // search K nearest neighbours
    K_neighbor_search search(*(reinterpret_cast<Tree*>(m_tree)), query, K);
    for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
      std::cout << " d(q, nearest neighbor) =  "
		<< tr_dist.inverse_of_transformed_distance(it->second) << " point: " 
		<< boost::get<0>(it->first)<< " index " << boost::get<1>(it->first) << std::endl;
    }
#endif
    }


