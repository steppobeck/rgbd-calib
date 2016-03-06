#ifndef RGBD_CALIB_OPENCVCHESSBOARDCORNERDETECTOR_H
#define RGBD_CALIB_OPENCVCHESSBOARDCORNERDETECTOR_H

#include <DataTypes.hpp>

#include <opencv/cv.h>
#include <vector>

  class OpenCVChessboardCornerDetector{

  public:
    OpenCVChessboardCornerDetector(unsigned width, unsigned height, int depth /*bits per channel*/, int channels, unsigned board_w, unsigned board_h, bool showimages = true);
    ~OpenCVChessboardCornerDetector();

    bool process(const void*, unsigned bytes, bool showimages = true);


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
    CvSize m_board_sz;
    CvPoint2D32f* m_corners;
    unsigned m_board_w;
    unsigned m_board_h;
    unsigned m_num_corners;


  public:
    std::vector<uv> corners;
  };




#endif // #ifndef RGBD_CALIB_OPENCVCHESSBOARDCORNERDETECTOR_H

