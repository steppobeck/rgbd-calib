#include "OpenCVChessboardCornerDetector.hpp"



#include <OpenCVUndistortion.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>


#include <iostream>
#include <sstream> //for std::stringstream 
#include <string>  //for std::string

/*static*/ std::string OpenCVChessboardCornerDetector::s_window_name = "";

OpenCVChessboardCornerDetector::OpenCVChessboardCornerDetector(unsigned width, unsigned height, int depth /*bits per channel*/, int channels, bool showimages, OpenCVUndistortion* undist, bool try_detect)
    : m_channels(channels),
      m_width(width),
      m_height(height),
      m_depth(depth),
      m_bytes(width * height * (depth/8) * channels),
      m_image(),
      m_gray_image(),
      m_gray_image_f(),
      m_tmp_image(),
      m_undist(undist),
      m_try_detect(try_detect),
      corners(),
      UL(0),
      UR(0),
      LL(0),
      LR(0),
      m_window_name("")
  {

    if("" == s_window_name){
      const void * address = static_cast<const void*>(this);
      std::stringstream ss;
      ss << address;  
      m_window_name = ss.str();
    }
    else{
      m_window_name = s_window_name;
    }



    if(1 != m_channels){
      m_image = cvCreateImage(cvSize(width,height), depth, channels);
      m_gray_image = cvCreateImage(cvSize(width,height), depth, 1);
      if(showimages){
	cvNamedWindow(m_window_name.c_str(), CV_WINDOW_AUTOSIZE);
      }
    }
    else{
      m_tmp_image = cvCreateImage(cvSize(width,height), depth, 1);
      m_gray_image = cvCreateImage(cvSize(1*width,1*height), depth, 1);
      m_gray_image_f = cvCreateImage(cvSize(1*width,1*height), depth, 1);
      if(showimages){
	cvNamedWindow(m_window_name.c_str(), CV_WINDOW_AUTOSIZE);
      }
    }

  }


  OpenCVChessboardCornerDetector::~OpenCVChessboardCornerDetector(){
    if(1 != m_channels)    
      cvReleaseImage(&m_image);
    cvReleaseImage(&m_gray_image);
    cvReleaseImage(&m_gray_image_f);
  }

  bool
  OpenCVChessboardCornerDetector::process(void* buffer, unsigned bytes, unsigned board_w, unsigned board_h, bool showimages){
    

    
    if(m_undist){
      buffer = m_undist->process(buffer, bytes);
    }

    if(1 != m_channels){
      //std::cerr << this << " using color image " << std::endl;
      memcpy(m_image->imageData, buffer, bytes);
      if(3 == m_channels){
	cvCvtColor( m_image, m_gray_image, CV_RGB2GRAY );
      }
      else if(4 == m_channels){
	cvCvtColor( m_image, m_gray_image, CV_RGBA2GRAY );
      }
      else{
	std::cerr << "ERROR in OpenCVChessboardCornerDetector::process unsupported number of channels" << std::endl;
	return false;
      }
    }
    else{
      // without median filter which is the default for Kinect V2 sensors
      memcpy(m_gray_image->imageData, buffer, bytes);
    }


    // clear latest corners
    corners.clear();
    bool result = false;

    // setup variables for corner detection
    const CvSize l_board_sz(cvSize( board_w, board_h ));
    CvPoint2D32f* l_corners = new CvPoint2D32f [ board_w * board_h ];
    const unsigned l_num_corners(board_w * board_h);      

    // now the image is in the gray_image
    int corner_count = 0;
    int found = 0;
    if(m_try_detect){
      found = cvFindChessboardCorners( m_gray_image, l_board_sz, l_corners,
				       &corner_count, /*CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE*/
				       CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    }
    //std::cerr << found << " " << corner_count << std::endl;
    // Get subpixel accuracy on those corners
    if((found != 0) && (corner_count == l_num_corners)){			
      cvFindCornerSubPix( m_gray_image, l_corners, corner_count, cvSize( 5, 5 ), 
			  cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    }

    // If we got a good board, add it to our data
    if((found != 0) && (corner_count == l_num_corners)){

      // copy from OpenCV to out corners
      for( unsigned j=0; j < l_num_corners; ++j ){
	uv c;
	c.u = l_corners[j].x;
	c.v = l_corners[j].y;
	corners.push_back(c);
      }
    
      // setup corner ids of extrema:
      /*
	UL - UR
	|     |
	LL - LR
      */
      UL = 0;
      UR = board_w - 1;
      LL = (board_w * board_h) - board_w;
      LR = board_w * board_h - 1;

      // detectFlips here!
      bool is_correct_orientation = ((corners[UR].u - corners[UL].u) > 0.0) && ((corners[LL].v - corners[UL].v) > 0.0);
      // correct flip
      if(!is_correct_orientation){
	std::cout << "INFO: OpenCVChessboardCornerDetector::process detected flip! trying to reflip" << std::endl;
	//std::cout << "INFO: OpenCVChessboardCornerDetector::process before flip:" << std::endl;
	std::vector<uv> corners_tmp;
	for(int y = (board_h - 1); y >= 0; --y){
	  for(int x = (board_w - 1); x >= 0; --x){
	    const unsigned c_id = y * board_w + x;
	    //std::cout << c_id << " -> " << corners[c_id] << " | ";
	    corners_tmp.push_back(corners[c_id]);
	  }
	  //std::cout << std::endl;
	}
	corners = corners_tmp;
	//std::cout << "INFO: OpenCVChessboardCornerDetector::process after flip:" << std::endl;
	for(int y = 0; y < board_h; ++y){
	  for(int x = 0; x < board_w; ++x){
	    const unsigned c_id = y * board_w + x;
	    //std::cout << c_id << " -> " << corners[c_id] << " | ";
	    corners_tmp.push_back(corners[c_id]);
	  }
	  //std::cout << std::endl;
	}
	//std::cout << "INFO: OpenCVChessboardCornerDetector::process end of flip:" << std::endl;

	// copy from our corners to OpenCV, needed in for showimages
	for( unsigned j=0; j < l_num_corners; ++j ){
	  l_corners[j].x = corners[j].u;
	  l_corners[j].y = corners[j].v;
	}
      }
      result = true;
    }

    
    if(showimages){
      
      // draw corners if found and correct number was found
      if((found != 0) && (corner_count == l_num_corners)){
	cvDrawChessboardCorners( m_gray_image, l_board_sz, l_corners, corner_count, found );
      }

      if(1 != m_channels){
	// original
	//cvShowImage( m_window_name.c_str(), m_gray_image);
	
	// needed for talk
	
	IplImage* tmp_image = cvCreateImage(cvSize(m_width,m_height), m_depth, m_channels);
	cvCvtColor( m_image, tmp_image, CV_BGR2RGB );
	cvDrawChessboardCorners( tmp_image, l_board_sz, l_corners, corner_count, found );
	//cvShowImage( m_window_name.c_str(), tmp_image);


	IplImage* rotated = cvCreateImage(cvSize(m_height, m_width), m_depth, m_channels);
	cvTranspose(tmp_image, rotated);
	cvFlip(rotated, NULL, 1);
	cvShowImage( m_window_name.c_str(), rotated);
	cvReleaseImage(&rotated);

	cvReleaseImage(&tmp_image);
      }
      else{
	//cvShowImage( m_window_name.c_str(), m_gray_image);
	IplImage* rotated = cvCreateImage(cvSize(m_height, m_width), m_depth, m_channels);
	cvTranspose(m_gray_image, rotated);
	cvFlip(rotated, NULL, 1);
	cvShowImage( m_window_name.c_str(), rotated);
	cvReleaseImage(&rotated);
      }
      cvWaitKey(10);
    }



    delete [] l_corners;
    return result;
  }


#if 0

#if 1
      // without median filter which is the default for Kinect V2 sensors
      memcpy(m_gray_image->imageData, buffer, bytes);
#else
      // with median filter which is used for Kinect V1 due to infrared pattern
      memcpy(m_tmp_image->imageData, buffer, bytes);
      cvResize(m_tmp_image, m_gray_image_f);
      cvSmooth(m_gray_image_f, m_gray_image, CV_MEDIAN, 5);
      std::cerr << "using median" << std::endl;
#endif

      // this is needed by Kinect V1
      //std::cerr << "NOTE: if KinectV1 need to smooth and translate the corners from IR to depth by ir_x += 5 and ir_y += 4" << std::endl;
      if(1 == m_channels){
	c.u += 5;
	c.v += 4;
      }
#endif




unsigned
OpenCVChessboardCornerDetector::getWidth() const{
  return m_width;
}

unsigned
OpenCVChessboardCornerDetector::getHeight() const{
  return m_height;
}
