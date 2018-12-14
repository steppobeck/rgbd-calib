#ifndef LPCNEARESTNEIGHBOURSEARCH_HPP
#define LPCNEARESTNEIGHBOURSEARCH_HPP


#include <LPCDataTypes.hpp>


#include <vector>


  class LPCNearestNeighbourSearch{

  public:

    LPCNearestNeighbourSearch(const std::vector<surfel>* surfels);
    ~LPCNearestNeighbourSearch();

    std::vector<surfel> search(const surfel& s,unsigned num_neighbours) const;
    std::vector<size_t> search_id(const surfel& s,unsigned num_neighbours) const;

  private:
    const std::vector<surfel>* m_surfels;
    void* m_tree;

  };


#endif // #ifndef LPCNEARESTNEIGHBOURSEARCH_HPP

