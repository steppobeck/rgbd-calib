#include "OneEuroFilterContainer.hpp"
#include <OneEuroFilter.hpp>


OneEuroFilterContainer::OneEuroFilterContainer(unsigned num_channels, unsigned num_values)
  : m_fc()
{
  for(unsigned c = 0; c != num_channels; ++c){
    m_fc.push_back(std::vector<OneEuroFilter*>());
    for(unsigned i = 0; i != num_values; ++i){
      m_fc[c].push_back(0);
    }
  }
}


OneEuroFilterContainer::~OneEuroFilterContainer(){
  for(unsigned c = 0; c != m_fc.size(); ++c){
    for(unsigned i = 0; i != m_fc[c].size(); ++i){
      delete m_fc[c][i];
    }
  }
}

void
OneEuroFilterContainer::init(unsigned channel, unsigned value_id,
			     double freq, double mincutoff, double beta, double dcutoff){
  if(m_fc[channel][value_id] != 0){
    delete m_fc[channel][value_id];
  }
  m_fc[channel][value_id] = new OneEuroFilter(freq, mincutoff, beta, dcutoff);
}


double
OneEuroFilterContainer::filter(unsigned channel, unsigned value_id, double value){
  return m_fc[channel][value_id]->filter(value);
}

