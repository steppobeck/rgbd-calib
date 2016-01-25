#ifndef KINECT_NEARESTNEIGHBOURSEARCH_HPP
#define KINECT_NEARESTNEIGHBOURSEARCH_HPP


#include <NaturalNeighbourInterpolator.hpp>


#include <vector>


  class NearestNeighbourSearch{

  public:

    NearestNeighbourSearch(const std::vector<nniSample>& spoints);
    ~NearestNeighbourSearch();

    std::vector<nniSample> search(const nniSample& ipolant,unsigned num_neighbours) const;

  private:
    const std::vector<nniSample>& shared_spoints;
    void* m_tree;

  };


#endif // #ifndef KINECT_NEARESTNEIGHBOURSEARCH_HPP

