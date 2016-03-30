#ifndef RGBD_CALIB_OPENCVUNDISTORTION_HPP
#define RGBD_CALIB_OPENCVUNDISTORTION_HPP

#include <opencv/cv.h>


class OpenCVUndistortion{
  
public:
  OpenCVUndistortion(unsigned width, unsigned height, int depth /*bits per channel*/, int channels,
		     float* intrinsic, float* distortion);
  ~OpenCVUndistortion();
  
  void* process(void*, bool irkv1 = false);
  

private:
  unsigned m_channels;
  unsigned m_depth;
  unsigned m_bytes;
  CvMat* m_intrinsic_matrix;
  CvMat* m_distortion_coeffs;
  CvMat* m_trans;
  IplImage* m_tmp_image;
  IplImage* m_distorted_image;
  IplImage* m_undistorted_image;
  IplImage* m_mapx;
  IplImage* m_mapy;
};


#endif // #ifndef  RGBD_CALIB_OPENCVUNDISTORTION_HPP
