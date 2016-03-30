#include "OpenCVUndistortion.hpp"
#include <timevalue.hpp>
#include <clock.hpp>
#include <iostream>



OpenCVUndistortion::OpenCVUndistortion(unsigned width, unsigned height, int depth /*bits per channel*/, int channels, float* intrinsic, float* distortion)
  : m_channels(channels),
    m_depth(depth),
    m_bytes(width * height * (depth/8) * channels),
    m_intrinsic_matrix(),
    m_distortion_coeffs(),
    m_trans(),
    m_tmp_image(),
    m_distorted_image(),
    m_undistorted_image(),
    m_mapx(),
    m_mapy()
{

  m_intrinsic_matrix   = cvCreateMat(3,3,CV_32FC1);
  memcpy(m_intrinsic_matrix->data.fl, intrinsic, 9 * sizeof(float));
    
  m_distortion_coeffs  = cvCreateMat(5,1,CV_32FC1);
  memcpy(m_distortion_coeffs->data.fl, distortion, 5 * sizeof(float));

  m_trans = cvCreateMat(2,3,CV_32FC1);
#if 0
  m_trans->data.fl[0] = 1.0;
  m_trans->data.fl[1] = 0.0;
  m_trans->data.fl[2] = 0.0;
  m_trans->data.fl[3] = 1.0;
  m_trans->data.fl[4] = -4.8;
  m_trans->data.fl[5] = -3.9;
#endif
  m_trans->data.fl[0] = 1.0;
  m_trans->data.fl[1] = 0.0;
  m_trans->data.fl[2] = -4.8;
  m_trans->data.fl[3] = 0.0;
  m_trans->data.fl[4] = 1.0;
  m_trans->data.fl[5] = -3.9;


  m_distorted_image = cvCreateImage(cvSize(width,height), depth, channels);
  m_undistorted_image = cvCreateImage(cvSize(width,height), depth, channels);
  m_tmp_image = cvCreateImage(cvSize(width,height), depth, channels);
  
  m_mapx = cvCreateImage( cvGetSize(m_distorted_image), IPL_DEPTH_32F, 1 );
  m_mapy = cvCreateImage( cvGetSize(m_distorted_image), IPL_DEPTH_32F, 1 );
  cvInitUndistortMap(m_intrinsic_matrix,m_distortion_coeffs,m_mapx,m_mapy);
  
}


OpenCVUndistortion::~OpenCVUndistortion(){
  cvReleaseMat(&m_intrinsic_matrix);
  cvReleaseMat(&m_distortion_coeffs);
  cvReleaseImage(&m_distorted_image);
  cvReleaseImage(&m_undistorted_image);
  cvReleaseImage(&m_mapx);
  cvReleaseImage(&m_mapy);
}

void*
OpenCVUndistortion::process(void* buffer, bool irkv1){

#if 0
  sensor::timevalue ts_start(sensor::clock::time());
#endif    


  if(irkv1){
    memcpy(m_tmp_image->imageData, buffer, m_bytes);
    cvWarpAffine(m_tmp_image/* CvArr* src*/, m_distorted_image/*CvArr* dst*/, m_trans
		 /*, int flags=CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, CvScalar fillval=cvScalarAll(0) */);
  }
  else{
    memcpy(m_distorted_image->imageData, buffer, m_bytes);
  }
  
  if(0){
    //cvFlip(m_distorted_image, m_distorted_image,0);
  }
  
  cvRemap(m_distorted_image, m_undistorted_image, m_mapx, m_mapy);
  
  if(0){
    //cvFlip(m_undistorted_image, m_undistorted_image,0);
  }

#if 0
  sensor::timevalue ts_elapsed(sensor::clock::time() - ts_start);
  std::cerr << ts_elapsed.msec() << std::endl;
#endif

  return m_undistorted_image->imageData;
}


