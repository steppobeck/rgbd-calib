#ifndef RGBD_CALIB_ONEEUROFILTERCONTAINER_HPP
#define RGBD_CALIB_ONEEUROFILTERCONTAINER_HPP

#include <vector>

class OneEuroFilter;

class OneEuroFilterContainer{

public:
  OneEuroFilterContainer(unsigned num_channels /* e.g. 2 or 3 */, unsigned num_values /* e.g. 35 */);

  ~OneEuroFilterContainer();

  void init(unsigned channel, unsigned value_id,
	    double freq, double mincutoff=1.0, double beta=0.0, double dcutoff=1.0);


  double filter(unsigned channel, unsigned value_id, double value);

private:
  std::vector< std::vector<OneEuroFilter*> > m_fc;

};



#endif // #ifndef RGBD_CALIB_ONEEUROFILTERCONTAINER_HPP
