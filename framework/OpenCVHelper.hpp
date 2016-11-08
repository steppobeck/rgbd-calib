#ifndef RGBD_CALIB_OPENCV_HELPER_HPP
#define RGBD_CALIB_OPENCV_HELPER_HPP


#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <string>

/*! Prepare a chessboard calibration pattern for OpenCV calibrateCamera. */
void calibrationPattern(std::vector< std::vector<cv::Point3f> >& output,
                        int pattern_width,
                        int pattern_height,
                        float square_size,
                        int nb_images);
void calibrationPattern(std::vector<cv::Point3f> & output,
                         int pattern_width,
                         int pattern_height,
                         float square_size);

double computeError(const cv::Mat& F,
                    const std::vector<std::vector<cv::Point2f> >& rgb_corners,
                    const std::vector<std::vector<cv::Point2f> >& depth_corners);


void writeMatrix(cv::FileStorage& output_file, const std::string& name, const cv::Mat& matrix);

void readMatrix(cv::FileStorage& input_file, const std::string& name, cv::Mat& matrix);

#endif // #ifndef RGBD_CALIB_OPENCV_HELPER_HPP
