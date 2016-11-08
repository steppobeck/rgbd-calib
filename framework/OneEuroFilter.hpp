#ifndef RGBD_CALIB_ONEEUROFILTER_HPP
#define RGBD_CALIB_ONEEUROFILTER_HPP

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>

// -----------------------------------------------------------------
class LowPassFilter {

public:
    LowPassFilter(double alpha)
        : mLastValue(0.0), mFilteredValue(0.0)
    {
        setAlpha(alpha) ;
        mInitialized = false ;
    }

    double filter(double value) {
        double result;
        if (mInitialized)
            result = mAlpha*value + (1.0-mAlpha) * mFilteredValue ;
        else {
            result = value;
            mInitialized = true;
        }
        mLastValue = value;
        mFilteredValue = result;
        return result;
    }


    double filterWithAlpha(double value, double alpha) {
        setAlpha(alpha) ;
        return filter(value) ;
    }

    bool hasLastRawValue(void) {
        return mInitialized ;
    }

    double lastRawValue(void) {
        return mLastValue ;
    }

private:

    double mLastValue, mAlpha, mFilteredValue ;
    bool mInitialized ;

    void setAlpha(double alpha) {
        if (alpha<=0.0 || alpha>1.0)
            throw std::range_error("alpha should be in (0.0., 1.0]") ;
        mAlpha = alpha ;
    }
};
// -----------------------------------------------------------------

// -----------------------------------------------------------------
class OneEuroFilter {

public:


  OneEuroFilter(double freq, double mincutoff=1.0, double beta_=0.0, double dcutoff=1.0) {
    setFrequency(freq) ;
    setMinCutoff(mincutoff) ;
    setBeta(beta_) ;
    setDerivateCutoff(dcutoff) ;
    mXFilt = new LowPassFilter(alpha(mincutoff)) ;
    mDXFilt = new LowPassFilter(alpha(dcutoff)) ;
  }

  ~OneEuroFilter(){
    delete mXFilt;
    delete mDXFilt;
  }


  double filter(double value) {
    // estimate the current variation per second
    double dvalue;
    if(!mXFilt->hasLastRawValue()){
      dvalue = value;
    } else {
      dvalue = (value - mXFilt->lastRawValue())*mRate;
    }
    
    double edvalue = mDXFilt->filterWithAlpha(dvalue, alpha(mDCutoff)) ;
    // use it to update the cutoff frequency
    double cutoff = mMinCutoff + mBeta*fabs(edvalue) ;
    // filter the given value
    return mXFilt->filterWithAlpha(value, alpha(cutoff)) ;
  }


private:

  double mRate ;
  double mMinCutoff ;
  double mBeta ;
  double mDCutoff ;
  LowPassFilter *mXFilt ;
  LowPassFilter *mDXFilt ;
  
  double alpha(double cutoff) {
    double te = 1.0 / mRate ;
    double tau = 1.0 / (2*M_PI*cutoff) ;
    return 1.0 / (1.0 + tau/te) ;
  }

  void setFrequency(double f) {
    if (f<=0) throw std::range_error("freq should be >0") ;
    mRate = f ;
  }
  
  void setMinCutoff(double mc) {
    if (mc<=0) throw std::range_error("mincutoff should be >0") ;
    mMinCutoff = mc ;
  }

  void setBeta(double b) {
    mBeta = b ;
  }
  
  void setDerivateCutoff(double dc) {
    if (dc<=0) throw std::range_error("dcutoff should be >0") ;
    mDCutoff = dc ;
  }
};

// -----------------------------------------------------------------






#endif // #ifndef RGBD_CALIB_ONEEUROFILTER_H


