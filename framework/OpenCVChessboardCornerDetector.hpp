#ifndef RGBD_CALIB_OPENCVCHESSBOARDCORNERDETECTOR_H
#define RGBD_CALIB_OPENCVCHESSBOARDCORNERDETECTOR_H

#include <DataTypes.hpp>

#include <opencv/cv.h>
#include <vector>

  class OpenCVUndistortion;

  class OpenCVChessboardCornerDetector{

  public:
    OpenCVChessboardCornerDetector(unsigned width, unsigned height, int depth /*bits per channel*/, int channels, bool showimages, OpenCVUndistortion* undist = 0);
    ~OpenCVChessboardCornerDetector();

    bool process(void*, unsigned bytes, unsigned board_w, unsigned board_h, bool showimages = true);


  private:
    unsigned m_channels;
    unsigned m_width;
    unsigned m_height;
    unsigned m_depth;
    unsigned m_bytes;
    IplImage* m_image;
    IplImage* m_gray_image;
    IplImage* m_gray_image_f;
    IplImage* m_tmp_image;
    OpenCVUndistortion* m_undist;

  public:
    std::vector<uv> corners;
  };




#endif // #ifndef RGBD_CALIB_OPENCVCHESSBOARDCORNERDETECTOR_H

