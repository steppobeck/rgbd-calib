#include "ChessboardSampling.hpp"
#include <OpenCVChessboardCornerDetector.hpp>
#include <MatrixInterpolation.hpp>

#include <sensor.hpp>
#include <timevalue.hpp>
#include <devicemanager.hpp>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include <unistd.h>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <limits>
namespace{



  void calcMeanSD(std::vector<float>& values, double& mean, double& stdev){

    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    mean = sum / values.size();
      
    const double sq_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0);
    stdev = std::sqrt(sq_sum / values.size() - mean * mean);

  }



  size_t calcNumFrames(std::ifstream& f, size_t fs){
    f.seekg(0,std::ios::end);
    const unsigned number_of_frames = (f.tellg()/fs);
    f.seekg(0, std::ios::beg);
    return number_of_frames;
  }




  struct DistId{
    float dist;
    unsigned id;
  };

  bool operator < (const DistId& a, const DistId& b){
    return a.dist < b.dist;
  }


}



  float computeAverageDist(xyz* a, xyz* b){
    float dx = 0.0;
    float dy = 0.0;
    for(unsigned i = 0; i < CB_WIDTH * CB_HEIGHT; ++i){
      dx += std::abs(a[i].x - b[i].x);
      dy += std::abs(a[i].y - b[i].y);
    }
    dx /= (CB_WIDTH * CB_HEIGHT);
    dy /= (CB_WIDTH * CB_HEIGHT);
    return std::sqrt(dx * dx + dy * dy);
  }

  float computeAverageDist(uv* a, uv* b){
    float dx = 0.0;
    float dy = 0.0;
    for(unsigned i = 0; i < CB_WIDTH * CB_HEIGHT; ++i){
      dx += std::abs(a[i].u - b[i].u);
      dy += std::abs(a[i].v - b[i].v);
    }
    dx /= (CB_WIDTH * CB_HEIGHT);
    dy /= (CB_WIDTH * CB_HEIGHT);
    return std::sqrt(dx * dx + dy * dy);
  }


  std::ostream& operator << (std::ostream& o, const ChessboardViewRGB& v){
    o << "ChessboardViewRGB time stamp: " << v.time << std::endl;
    o << "ChessboardViewRGB corners:" << std::endl;
    for(unsigned i = 0; i< CB_WIDTH * CB_HEIGHT; ++i){
      o << i << " -> " << v.corners[i] << std::endl;
    }
    return o;
  }

  std::ostream& operator << (std::ostream& o, const ChessboardViewIR& v){
    o << "ChessboardViewIR time stamp: " << v.time << std::endl;
    o << "ChessboardViewIR corners:" << std::endl;
    for(unsigned i = 0; i< CB_WIDTH * CB_HEIGHT; ++i){
      o << i << " -> " << v.corners[i] << std::endl;
    }
    return o;
  }

  std::ostream& operator << (std::ostream& o, const ChessboardPose& v){
    o << "ChessboardPose time stamp: " << v.time << std::endl << " mat: " << v.mat;
    return o;
  }

  ChessboardViewRGB interpolate(const ChessboardViewRGB& a, const ChessboardViewRGB& b, float t){
    ChessboardViewRGB res;
    // r.u = (1.0f - t)*a.u + t*b.u;
    res.time = (1.0f - t) * a.time + t * b.time;
    for(unsigned i = 0; i < CB_WIDTH*CB_HEIGHT; ++i){
      res.corners[i] = interpolate(a.corners[i], b.corners[i], t);
    }
    return res;
  }

  ChessboardViewIR interpolate(const ChessboardViewIR& a, const ChessboardViewIR& b, float t){
    ChessboardViewIR res;
    // r.u = (1.0f - t)*a.u + t*b.u;
    res.time = (1.0f - t) * a.time + t * b.time;
    for(unsigned i = 0; i < CB_WIDTH*CB_HEIGHT; ++i){
      res.corners[i] = interpolate(a.corners[i], b.corners[i], t);
    }
    return res;
  }


  ChessboardSampling::ChessboardSampling(const char* filenamebase)
    : m_filenamebase(filenamebase),
      m_poses(),
      m_cb_rgb(),
      m_cb_ir()
  {}


  ChessboardSampling::~ChessboardSampling()
  {}


  bool
  ChessboardSampling::init(bool reload){
    bool res = false;
    if(reload){
      res = loadRecording();
      res = saveChessboards();
    }
    else{
      res = loadChessboards();
    }
    res = loadPoses();
    return res;
  }

  void
  ChessboardSampling::interactiveShow(){
    bool res = false;
    res = loadPoses();
    res = showRecordingAndPoses();
  }


  double
  ChessboardSampling::searchSlowestTime(double starttime){
    
    std::vector<DistId> dists;
    for(unsigned i = 1; i < m_poses.size(); ++i){

      glm::vec4 last_pose = m_poses[i - 1].mat * glm::vec4(0.0,0.0,0.0,1.0);
      glm::vec4 curr_pose = m_poses[i].mat * glm::vec4(0.0,0.0,0.0,1.0);
      float curr_dist = glm::length((glm::vec3(curr_pose.x, curr_pose.y, curr_pose.z)
				     - glm::vec3(last_pose.x,last_pose.y,last_pose.z)));
      
      DistId di;
      di.dist = curr_dist;
      di.id = i;
      if(m_poses[i].time > starttime){
	dists.push_back(di);
      }

    }

    std::sort(dists.begin(), dists.end());
    const double time = m_poses[dists.begin()->id].time;
    std::cerr << "slowest pose at time: " << time << std::endl
	      << "ChessboardPose: " << m_poses[dists.begin()->id] << std::endl;

    return time;
  }


  glm::mat4
  ChessboardSampling::interpolatePose(double time, bool& valid){
    unsigned a = 0;
    unsigned b = 0;
    for(unsigned i = 1; i < m_poses.size(); ++i){
      const double curr(m_poses[i].time);
      if(curr > time){
	a = i - 1;
	b = i;
	break;
      }
    }
    if(a == b){
      std::cerr << "ERROR in ChessboardSampling::interpolatePose -> could not find for time "
		<< time << std::endl;
      valid = false;
      return glm::mat4();
    }
    valid = true;
    const float t = (time - m_poses[a].time)/(m_poses[b].time - m_poses[a].time);
    return interpolate(m_poses[a].mat, m_poses[b].mat, t);
  }


  double
  ChessboardSampling::getPoseSpeed(double time, bool& valid){
    unsigned a = 0;
    unsigned b = 0;
    for(unsigned i = 1; i < m_poses.size(); ++i){
      const double curr(m_poses[i].time);
      if(curr > time){
	a = i - 1;
	b = i;
	break;
      }
    }
    if(a == b){
      std::cerr << "ERROR in ChessboardSampling::getPoseSpeed -> could not find for time "
		<< time << std::endl;
      valid = false;
      return std::numeric_limits<double>::max();
    }
    valid = true;

    glm::vec4 last_pose = m_poses[b].mat * glm::vec4(0.0,0.0,0.0,1.0);
    glm::vec4 curr_pose = m_poses[a].mat * glm::vec4(0.0,0.0,0.0,1.0);
    return glm::length((glm::vec3(curr_pose.x, curr_pose.y, curr_pose.z)
			- glm::vec3(last_pose.x,last_pose.y,last_pose.z)));
#if 0
    gloost::Point3 last_pose = m_poses[b].mat * gloost::Point3(0.0,0.0,0.0);
    gloost::Point3 curr_pose = m_poses[a].mat * gloost::Point3(0.0,0.0,0.0);
    return (curr_pose - last_pose).length();
#endif
  }


  ChessboardViewRGB
  ChessboardSampling::interpolateRGB(double time, bool& valid){
    unsigned a = 0;
    unsigned b = 0;
    for(unsigned i = 1; i < m_cb_rgb.size(); ++i){
      const double curr(m_cb_rgb[i].time);
      if(curr > time){
	a = i - 1;
	b = i;
	break;
      }
    }
    if(a == b){
      std::cerr << "ERROR in ChessboardSampling::interpolateRGB -> could not find for time "
		<< time << std::endl;
      valid = false;
      return ChessboardViewRGB();
    }
    valid = true;
    const float t = (time - m_cb_rgb[a].time)/(m_cb_rgb[b].time - m_cb_rgb[a].time);
    return interpolate(m_cb_rgb[a], m_cb_rgb[b], t);
  }



  ChessboardViewIR
  ChessboardSampling::interpolateIR(double time, bool& valid){
    unsigned a = 0;
    unsigned b = 0;
    for(unsigned i = 1; i < m_cb_ir.size(); ++i){
      const double curr(m_cb_ir[i].time);
      if(curr > time){
	a = i - 1;
	b = i;
	break;
      }
    }
    if(a == b){
      std::cerr << "ERROR in ChessboardSampling::interpolateIR -> could not find for time "
		<< time << std::endl;
      valid = false;
      return ChessboardViewIR();
    }
    valid = true;
    const float t = (time - m_cb_ir[a].time)/(m_cb_ir[b].time - m_cb_ir[a].time);
    return interpolate(m_cb_ir[a], m_cb_ir[b], t);
  }



  void
  ChessboardSampling::dump(){

    std::cout << "RGB -----------------------------------------------------" << std::endl;
    for(unsigned i = 0; i < m_cb_rgb.size(); ++i){
      std::cout << m_cb_rgb[i] << std::endl;
    }
    std::cout << "RGB frequencies in Hz -----------------------------------------------------" << std::endl;
    for(unsigned i = 1; i < m_cb_rgb.size(); ++i){
      std::cout << 1000.0 / (1000.0 * (m_cb_rgb[i].time - m_cb_rgb[i-1].time)) << std::endl;
    }



    std::cout << "IR -----------------------------------------------------" << std::endl;
    for(unsigned i = 0; i < m_cb_ir.size(); ++i){
      std::cout << m_cb_ir[i] << std::endl;
    }
    std::cout << "IR frequencies in Hz -----------------------------------------------------" << std::endl;
    for(unsigned i = 1; i < m_cb_ir.size(); ++i){
      std::cout << 1000.0 / (1000.0 * (m_cb_ir[i].time - m_cb_ir[i-1].time)) << std::endl;
    }



    std::cout << "Poses -----------------------------------------------------" << std::endl;
    for(unsigned i = 0; i < m_poses.size(); ++i){
      std::cout << m_poses[i] << std::endl;
    }
    std::cout << "Poses frequencies in Hz -----------------------------------------------------" << std::endl;
    for(unsigned i = 1; i < m_poses.size(); ++i){
      std::cout << 1000.0 / (1000.0 * (m_poses[i].time - m_poses[i-1].time)) << std::endl;
    }


  }

  bool
  ChessboardSampling::loadPoses(){
    m_poses.clear();
    // load e.g. 23_sweep.pose
    std::string filename_poses(m_filenamebase + ".poses");
    std::cerr << "loading poses from file " << filename_poses << std::endl;
    std::ifstream infile(filename_poses.c_str(), std::ifstream::binary);
    const size_t num_poses = calcNumFrames(infile, sizeof(double) + sizeof(glm::mat4) + 8);
    unsigned padding[2]; // bytes
    for(size_t i = 0; i != num_poses; ++i){
      
      m_poses.push_back(ChessboardPose());
      infile.read((char*) &m_poses[i].time, sizeof(double));
      //std::cerr << m_poses[i].time << std::endl;
      infile.read((char*) glm::value_ptr(m_poses[i].mat), sizeof(glm::mat4));
      infile.read((char*) padding, 8);
      //std::cerr << m_poses[i].time << std::endl;
      //std::cerr << m_poses[i].mat << std::endl;
    }
    std::cerr << "ChessboardSampling::loadPoses() loaded poses: " << m_poses.size() << std::endl;
    return true;
  }


  bool
  ChessboardSampling::showRecordingAndPoses(){


    int port = 5000;
    int devicename = 7;
    sensor::device* d = sensor::devicemanager::the()->get_dtrack(port, sensor::timevalue::const_050_ms);
    sensor::sensor* s = new sensor::sensor(d,devicename/*station*/);
    
    glm::mat4 xmitterOffset;
    s->setTransmitterOffset(xmitterOffset);
    
    glm::mat4 receiverOffset;
    s->setReceiverOffset(receiverOffset);
    for(unsigned i = 0; i < 3; ++i){
      sleep(1);
      std::cerr << s->getMatrix() << std::endl;
    }
    glm::mat4 target_mat = s->getMatrix();


    // show all poses
    glm::vec4 ori(0.0,0.0,0.0,1.0);
    unsigned best_id = 0;
    float best_dist = std::numeric_limits<float>::max();
    std::cout << "Poses -----------------------------------------------------" << std::endl;
    for(unsigned i = 0; i < m_poses.size(); ++i){

      glm::vec4 last_pose = m_poses[i].mat * ori;
      glm::vec4 curr_pose = target_mat * ori;
      const float dist =  glm::length((glm::vec3(curr_pose.x, curr_pose.y, curr_pose.z)
				       - glm::vec3(last_pose.x,last_pose.y,last_pose.z)));
#if 0
      const float dist = glm::length((m_poses[i].mat * ori) - (target_mat * ori));
#endif
      std::cerr << dist << " " << best_dist << std::endl;
      if(dist < best_dist){
	best_dist =dist;
	best_id = i;
      }
    }
    std::cout << best_id << " -> " << m_poses[best_id] << std::endl;

    std::cout << "RGBD -----------------------------------------------------" << std::endl;

    cvNamedWindow("rgb", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("ir", CV_WINDOW_AUTOSIZE);

    IplImage* rgb = cvCreateImage(cvSize(1280,1080), 8, 3);
    float* depth = new float[512*424];
    IplImage* ir = cvCreateImage(cvSize(512,424), 8, 1);

    std::ifstream infile_fr(m_filenamebase.c_str(), std::ifstream::binary);
    const size_t num_frames = calcNumFrames(infile_fr, (2 * sizeof(double))
					    + (1280 * 1080 * 3)
					    + (512 * 424 * sizeof(float))
					    + (512 * 424));
    for(size_t i = 0; i != num_frames; ++i){
      double rgb_time;
      infile_fr.read((char*) &rgb_time, sizeof(double));
      infile_fr.read((char*) rgb->imageData, 1280*1080 * 3);

      double ir_time;
      infile_fr.read((char*) &ir_time, sizeof(double));
      infile_fr.read((char*) depth, 512 * 424 * sizeof(float));
      infile_fr.read((char*) ir->imageData, 512 * 424);

      std::cout << "rgb time: " << rgb_time << " ir time: " << ir_time << std::endl;

      // show rgb and ir image
      int key = -1;
      while(-1 == key){
	cvShowImage( "rgb", rgb);
	cvShowImage( "ir", ir);
	key = cvWaitKey(10);
      }

    }

    cvReleaseImage(&rgb);
    delete [] depth;
    cvReleaseImage(&ir);

  }


  bool
  ChessboardSampling::loadRecording(){

    m_cb_rgb.clear();
    m_cb_ir.clear();


    unsigned char* rgb = new unsigned char[1280*1080 * 3];
    float* depth = new float[512*424];
    unsigned char* ir = new unsigned char[512*424];



    OpenCVChessboardCornerDetector cd_c(1280,
					1080,
					8 /*bits per channel*/,
					3 /*num channels*/,
					CB_WIDTH, CB_HEIGHT);

    OpenCVChessboardCornerDetector cd_i(512,
					424,
					8 /*bits per channel*/,
					1,
					CB_WIDTH, CB_HEIGHT);




    std::ifstream infile_fr(m_filenamebase.c_str(), std::ifstream::binary);
    const size_t num_frames = calcNumFrames(infile_fr, (2 * sizeof(double))
					    + (1280 * 1080 * 3)
					    + (512 * 424 * sizeof(float))
					    + (512 * 424));
    for(size_t i = 0; i != num_frames; ++i){
      ChessboardViewRGB cb_rgb;
      infile_fr.read((char*) &cb_rgb.time, sizeof(double));
      infile_fr.read((char*) rgb, 1280*1080 * 3);

      ChessboardViewIR cb_ir;
      infile_fr.read((char*) &cb_ir.time, sizeof(double));
      infile_fr.read((char*) depth, 512 * 424 * sizeof(float));
      infile_fr.read((char*) ir, 512 * 424);

      // detect corners in color image
      bool found_color = cd_c.process((unsigned char*) rgb, 1280*1080 * 3, true);
      bool found_ir = cd_i.process((unsigned char*) ir, 512 * 424, true);
      
      if(found_color && found_ir &&
	 (cd_i.corners.size() == cd_c.corners.size() &&
	  (cd_i.corners.size() == CB_WIDTH * CB_HEIGHT))){

	for(unsigned c_id = 0; c_id != cd_c.corners.size(); ++c_id){
	  cb_rgb.corners[c_id].u = cd_c.corners[c_id].u;
	  cb_rgb.corners[c_id].v = cd_c.corners[c_id].v;

	  cb_ir.corners[c_id].x = cd_i.corners[c_id].u;
	  cb_ir.corners[c_id].y = cd_i.corners[c_id].v;
	  cb_ir.corners[c_id].z = getBilinear(depth, 512, 424,
					      cb_ir.corners[c_id].x,
					      cb_ir.corners[c_id].y); 
	}

	m_cb_rgb.push_back(cb_rgb);
	m_cb_ir.push_back(cb_ir);
      }
    }

    std::cerr << "ChessboardSampling::loadRecording() loaded chessboard views: "
	      << m_cb_rgb.size() << std::endl;

    delete [] rgb;
    delete [] depth;
    delete [] ir;


    return true;
  }

  bool
  ChessboardSampling::saveChessboards(){
    std::string fn(m_filenamebase + ".chessboardsrgb");
    std::ofstream f(fn.c_str(), std::ofstream::binary);
    f.write((const char*) &m_cb_rgb.front(), m_cb_rgb.size() * sizeof(ChessboardViewRGB));
    f.close();

    std::string fn2(m_filenamebase + ".chessboardsir");
    std::ofstream f2(fn2.c_str(), std::ofstream::binary);
    f2.write((const char*) &m_cb_ir.front(), m_cb_ir.size() * sizeof(ChessboardViewIR));
    f2.close();
    return true;
  }

  bool
  ChessboardSampling::loadChessboards(){
    std::string fn(m_filenamebase + ".chessboardsrgb");
    std::ifstream f(fn.c_str(), std::ifstream::binary);
    const size_t num_rgb = calcNumFrames(f, sizeof(ChessboardViewRGB));
    m_cb_rgb.clear();
    m_cb_rgb.resize(num_rgb);
    f.read((char*) &m_cb_rgb.front(), num_rgb * sizeof(ChessboardViewRGB));
    f.close();

    std::string fn2(m_filenamebase + ".chessboardsir");
    std::ifstream f2(fn2.c_str(), std::ifstream::binary);
    const size_t num_ir  = calcNumFrames(f2, sizeof(ChessboardViewIR));
    m_cb_ir.clear();
    m_cb_ir.resize(num_ir);
    f2.read((char*) &m_cb_ir.front(), num_ir * sizeof(ChessboardViewIR));
    f2.close();
    
    std::cerr << "ChessboardSampling::loadChessboards() loaded chessboard views: "
	      << m_cb_rgb.size() << " " << m_cb_ir.size() << std::endl;
    return true;
  }


  void
  ChessboardSampling::filterIR(float pose_offset){
    computeQualityIR(pose_offset);
  }

  double 
  ChessboardSampling::searchStartIR(){
    return m_cb_ir[10 < m_cb_ir.size() ? 10 : 0].time;
  }

  void
  ChessboardSampling::computeQualityIR(float pose_offset){

    const float max_speed = 0.01;
    const float min_speed = 0.0;

    // for each chessboardview
    for(unsigned i = 0; i < m_cb_ir.size(); ++i){

      const float time = m_cb_ir[i].time;
      bool valid_speed;
      const float pose_speed = getPoseSpeed(time + pose_offset, valid_speed);
      const float speed_quality = 1.0 - std::min(pose_speed, max_speed) / max_speed;
      //std::cerr << "pose speed at time " << time + pose_offset << " : " << pose_speed  << " -> " << speed_quality << std::endl;

      // 0. set quality to 1 for all corners
      for(unsigned c = 0; c < CB_WIDTH * CB_HEIGHT; ++c){
	m_cb_ir[i].quality[c] = speed_quality;
      }

#if 0
      // 1. Horizontal quality
      for(unsigned row = 0; row < CB_HEIGHT; ++row){
	const unsigned base_cid = CB_WIDTH * row;
	const unsigned one_past_end_cid = base_cid + CB_WIDTH;
	//gloost::Vector3 maindirection = calcMainDirection(m_cb_ir[i].corners, base_cid, one_past_end_cid,0);
	for(unsigned cid = base_cid; cid < one_past_end_cid; ++cid){
	  
	}
      }
#endif      
      

    }
  }

