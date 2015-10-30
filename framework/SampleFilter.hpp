#ifndef RGBD_CALIB_SAMPLE_FILTER_H
#define RGBD_CALIB_SAMPLE_FILTER_H


#include <DataTypes.hpp>

#include <vector>

  class SampleFilter{

  public:
    SampleFilter(unsigned bs);
    ~SampleFilter();

    void clear();

    void addSamples(std::vector<samplePoint>& board);

    std::vector<samplePoint> getFiltered();


  private:
    unsigned m_bs;
    std::vector<std::vector<samplePoint>*> m_s;
  };


#endif // #ifndef RGBD_CALIB_SAMPLE_FILTER_H

