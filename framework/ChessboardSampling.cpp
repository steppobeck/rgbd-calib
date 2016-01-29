

#include "ChessboardSampling.hpp"



#include <OpenCVChessboardCornerDetector.hpp>
#include <MatrixInterpolation.hpp>

#include <OneEuroFilterContainer.hpp>

#include <PlaneFit.hpp>

#include <CSVExporter.hpp>

#include <sensor.hpp>
#include <timevalue.hpp>
#include <devicemanager.hpp>

#include <glm/vec2.hpp>
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




#include <set>

#include <unistd.h>
#include <cmath>
#include <fstream>
#include <limits>


namespace{




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




  std::ostream& operator << (std::ostream& o, const ChessboardRange& v){

    o << "ChessboardRange: " << v.start << " -> " << v.end << std::endl;
    o << "frametime stats: " << v.avg_frametime << " , [" << v.sd_frametime << "] , (" << v.max_frametime << ") , {" << v.median_frametime << "}" << std::endl;
    return o;

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
    res.valid = a.valid && b.valid;
    for(unsigned i = 0; i < CB_WIDTH*CB_HEIGHT; ++i){
      res.corners[i] = interpolate(a.corners[i], b.corners[i], t);
      res.quality[i] = (1.0f - t) * a.quality[i] + t * b.quality[i];
    }
    return res;
  }

  ChessboardViewIR interpolate(const ChessboardViewIR& a, const ChessboardViewIR& b, float t){
    ChessboardViewIR res;
    // r.u = (1.0f - t)*a.u + t*b.u;
    res.time = (1.0f - t) * a.time + t * b.time;
    res.valid = a.valid && b.valid;
    for(unsigned i = 0; i < CB_WIDTH*CB_HEIGHT; ++i){
      res.corners[i] = interpolate(a.corners[i], b.corners[i], t);
      res.quality[i] = (1.0f - t) * a.quality[i] + t * b.quality[i];
    }
    return res;
  }


  ChessboardSampling::ChessboardSampling(const char* filenamebase)
    : m_filenamebase(filenamebase),
      m_poses(),
      m_cb_rgb(),
      m_cb_ir(),
      m_valid_ranges()
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
  ChessboardSampling::interactiveShow(unsigned start, unsigned end){
    bool res = false;
    res = loadPoses();
    res = showRecordingAndPoses(start, end);
  }


  double
  ChessboardSampling::searchSlowestTime(double starttime) const{
    
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
    return time;
  }


  glm::mat4
  ChessboardSampling::interpolatePose(double time, bool& valid) const{
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
  ChessboardSampling::interpolateRGB(double time, bool& valid) const{
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
    const float t = (time - m_cb_rgb[a].time)/(m_cb_rgb[b].time - m_cb_rgb[a].time);
    ChessboardViewRGB result = interpolate(m_cb_rgb[a], m_cb_rgb[b], t);
    valid = result.valid;
    return result;
  }



  ChessboardViewIR
  ChessboardSampling::interpolateIR(double time, bool& valid) const{
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
    
    const float t = (time - m_cb_ir[a].time)/(m_cb_ir[b].time - m_cb_ir[a].time);
    ChessboardViewIR result = interpolate(m_cb_ir[a], m_cb_ir[b], t);
    valid = result.valid;
    return result;
  }



  void
  ChessboardSampling::dump(){

    std::cout << "RGB -----------------------------------------------------" << std::endl;
    for(unsigned i = 0; i < m_cb_rgb.size(); ++i){
      std::cout << "cb_id: " << i << std::endl;
      std::cout << m_cb_rgb[i] << std::endl;
    }
    std::cout << "RGB frequencies in Hz -----------------------------------------------------" << std::endl;
    for(unsigned i = 1; i < m_cb_rgb.size(); ++i){
      std::cout << "cb_id: " << i << " " << 1000.0 / (1000.0 * (m_cb_rgb[i].time - m_cb_rgb[i-1].time)) << std::endl;
    }



    std::cout << "IR -----------------------------------------------------" << std::endl;
    for(unsigned i = 0; i < m_cb_ir.size(); ++i){
      std::cout << "cb_id: " << i << std::endl;
      std::cout << m_cb_ir[i] << std::endl;
    }
    std::cout << "IR frequencies in Hz -----------------------------------------------------" << std::endl;
    for(unsigned i = 1; i < m_cb_ir.size(); ++i){
      std::cout << "cb_id: " << i << " " << 1000.0 / (1000.0 * (m_cb_ir[i].time - m_cb_ir[i-1].time)) << std::endl;
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
    const size_t num_poses = calcNumFrames(infile, sizeof(double) + sizeof(glm::mat4));
    for(size_t i = 0; i != num_poses; ++i){
      
      m_poses.push_back(ChessboardPose());
      infile.read((char*) &m_poses[i].time, sizeof(double));
      //std::cerr << m_poses[i].time << std::endl;
      infile.read((char*) glm::value_ptr(m_poses[i].mat), sizeof(glm::mat4));
      //std::cerr << m_poses[i].time << std::endl;
      //std::cerr << m_poses[i].mat << std::endl;
    }
    std::cerr << "ChessboardSampling::loadPoses() loaded poses: " << m_poses.size() << std::endl;
    return true;
  }


  bool
  ChessboardSampling::showRecordingAndPoses(unsigned start, unsigned end){

#if 0
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
#endif


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


      if((end == 0) || (start < i) && (i < end)){
	std::cout << "cb_id: " << i << " rgb time: " << rgb_time << " ir time: " << ir_time << std::endl;

	// show rgb and ir image
	int key = -1;
	while(-1 == key){
	  cvShowImage( "rgb", rgb);
	  cvShowImage( "ir", ir);
	  key = cvWaitKey(10);
	}
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
      cb_rgb.valid = 1;
      infile_fr.read((char*) &cb_rgb.time, sizeof(double));
      infile_fr.read((char*) rgb, 1280*1080 * 3);

      ChessboardViewIR cb_ir;
      cb_ir.valid = 1;
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
      }
      else{
	cb_rgb.valid = 0;
	cb_ir.valid = 0;
      }
	
      m_cb_rgb.push_back(cb_rgb);
      m_cb_ir.push_back(cb_ir);

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
  ChessboardSampling::detectTimeJumpsInRanges(){

    std::set<unsigned> to_invalidate;
    for(const auto& r : m_valid_ranges){
      for(unsigned cb_id = r.start + 1; cb_id < r.end; ++cb_id){
	const float curr_frametime = m_cb_ir[cb_id].time - m_cb_ir[cb_id - 1].time;
	if(curr_frametime > (r.median_frametime + 3 * r.sd_frametime)){
	  to_invalidate.insert(cb_id);
	}
      }
    }

    for(const auto& cb_id : to_invalidate){
      invalidateAt(cb_id, 10);
    }

  }


  void
  ChessboardSampling::invalidateAt(unsigned cb_id, unsigned window_size){

    for(unsigned i = std::max(0, (int(cb_id) - int(window_size))); i < std::min(unsigned(m_cb_ir.size()), (cb_id + window_size + 1u)); ++i){
      m_cb_ir[i].valid = 0;
      m_cb_rgb[i].valid = 0;
    }

  }

  void
  ChessboardSampling::oneEuroFilterInRanges(){


    for(const auto& r : m_valid_ranges){

      OneEuroFilterContainer rgb_filter(2, CB_WIDTH * CB_HEIGHT);
      OneEuroFilterContainer ir_filter(3, CB_WIDTH * CB_HEIGHT);

      // configure one euro filters
      // 1. color corner
      const double rgb_freq(1.0/r.avg_frametime);
      const double rgb_mincutoff = 1.0; //
      const double rgb_beta = 0.007;    // cutoff slope
      const double rgb_dcutoff = 1.0;   // this one should be ok 
      for(unsigned i = 0; i != CB_WIDTH * CB_HEIGHT; ++i){
	rgb_filter.init(0, i, rgb_freq, rgb_mincutoff, rgb_beta, rgb_dcutoff);
	rgb_filter.init(1, i, rgb_freq, rgb_mincutoff, rgb_beta, rgb_dcutoff);
      }

      // 2. ir corner + ir depth
      const float ir_freq(1.0/r.avg_frametime);
      const double ir_mincutoff = 1.0; //
      const double ir_beta = 0.007;    // cutoff slope
      const double ir_dcutoff = 1.0;   // this one should be ok 
      
      const double ird_mincutoff = 1.0; //
      const double ird_beta = 0.007;    // cutoff slope
      const double ird_dcutoff = 1.0;   // this one should be ok 
      
      for(unsigned i = 0; i != CB_WIDTH * CB_HEIGHT; ++i){
	ir_filter.init(0, i, ir_freq, ir_mincutoff, ir_beta, ir_dcutoff);
	ir_filter.init(1, i, ir_freq, ir_mincutoff, ir_beta, ir_dcutoff);
	ir_filter.init(2, i, ir_freq, ird_mincutoff, ird_beta, ird_dcutoff);
      }


      
      for(unsigned cb_id = r.start; cb_id < r.end; ++cb_id){

	for(unsigned cid = 0; cid != CB_WIDTH * CB_HEIGHT; ++cid){
	  m_cb_rgb[cb_id].corners[cid].u = rgb_filter.filter(0 /*u*/,cid, m_cb_rgb[cb_id].corners[cid].u);
	  m_cb_rgb[cb_id].corners[cid].v = rgb_filter.filter(1 /*v*/,cid, m_cb_rgb[cb_id].corners[cid].v);
	  m_cb_ir[cb_id].corners[cid].x  =  ir_filter.filter(0 /*x*/,cid, m_cb_ir[cb_id].corners[cid].x);
	  m_cb_ir[cb_id].corners[cid].y  =  ir_filter.filter(1 /*y*/,cid, m_cb_ir[cb_id].corners[cid].y);
	  m_cb_ir[cb_id].corners[cid].z  =  ir_filter.filter(2 /*z*/,cid, m_cb_ir[cb_id].corners[cid].z);
	}

      }

    }
  }



  void
  ChessboardSampling::gatherCornerTracesInRanges(const char* prefix){
    CSVExporter csv(35 + 2*35 + 2*35);
    // z..., x... y... s... t...

    csv.slotnames.push_back("00z");
    csv.slotnames.push_back("01z");
    csv.slotnames.push_back("02z");
    csv.slotnames.push_back("03z");
    csv.slotnames.push_back("04z");
    csv.slotnames.push_back("05z");
    csv.slotnames.push_back("06z");
    csv.slotnames.push_back("07z");
    csv.slotnames.push_back("08z");
    csv.slotnames.push_back("09z");
    csv.slotnames.push_back("10z");
    csv.slotnames.push_back("11z");
    csv.slotnames.push_back("12z");
    csv.slotnames.push_back("13z");
    csv.slotnames.push_back("14z");
    csv.slotnames.push_back("15z");
    csv.slotnames.push_back("16z");
    csv.slotnames.push_back("17z");
    csv.slotnames.push_back("18z");
    csv.slotnames.push_back("19z");
    csv.slotnames.push_back("20z");
    csv.slotnames.push_back("21z");
    csv.slotnames.push_back("22z");
    csv.slotnames.push_back("23z");
    csv.slotnames.push_back("24z");
    csv.slotnames.push_back("25z");
    csv.slotnames.push_back("26z");
    csv.slotnames.push_back("27z");
    csv.slotnames.push_back("28z");
    csv.slotnames.push_back("29z");
    csv.slotnames.push_back("30z");
    csv.slotnames.push_back("31z");
    csv.slotnames.push_back("32z");
    csv.slotnames.push_back("33z");
    csv.slotnames.push_back("34z");


    csv.slotnames.push_back("00x");
    csv.slotnames.push_back("01x");
    csv.slotnames.push_back("02x");
    csv.slotnames.push_back("03x");
    csv.slotnames.push_back("04x");
    csv.slotnames.push_back("05x");
    csv.slotnames.push_back("06x");
    csv.slotnames.push_back("07x");
    csv.slotnames.push_back("08x");
    csv.slotnames.push_back("09x");
    csv.slotnames.push_back("10x");
    csv.slotnames.push_back("11x");
    csv.slotnames.push_back("12x");
    csv.slotnames.push_back("13x");
    csv.slotnames.push_back("14x");
    csv.slotnames.push_back("15x");
    csv.slotnames.push_back("16x");
    csv.slotnames.push_back("17x");
    csv.slotnames.push_back("18x");
    csv.slotnames.push_back("19x");
    csv.slotnames.push_back("20x");
    csv.slotnames.push_back("21x");
    csv.slotnames.push_back("22x");
    csv.slotnames.push_back("23x");
    csv.slotnames.push_back("24x");
    csv.slotnames.push_back("25x");
    csv.slotnames.push_back("26x");
    csv.slotnames.push_back("27x");
    csv.slotnames.push_back("28x");
    csv.slotnames.push_back("29x");
    csv.slotnames.push_back("30x");
    csv.slotnames.push_back("31x");
    csv.slotnames.push_back("32x");
    csv.slotnames.push_back("33x");
    csv.slotnames.push_back("34x");


    csv.slotnames.push_back("00y");
    csv.slotnames.push_back("01y");
    csv.slotnames.push_back("02y");
    csv.slotnames.push_back("03y");
    csv.slotnames.push_back("04y");
    csv.slotnames.push_back("05y");
    csv.slotnames.push_back("06y");
    csv.slotnames.push_back("07y");
    csv.slotnames.push_back("08y");
    csv.slotnames.push_back("09y");
    csv.slotnames.push_back("10y");
    csv.slotnames.push_back("11y");
    csv.slotnames.push_back("12y");
    csv.slotnames.push_back("13y");
    csv.slotnames.push_back("14y");
    csv.slotnames.push_back("15y");
    csv.slotnames.push_back("16y");
    csv.slotnames.push_back("17y");
    csv.slotnames.push_back("18y");
    csv.slotnames.push_back("19y");
    csv.slotnames.push_back("20y");
    csv.slotnames.push_back("21y");
    csv.slotnames.push_back("22y");
    csv.slotnames.push_back("23y");
    csv.slotnames.push_back("24y");
    csv.slotnames.push_back("25y");
    csv.slotnames.push_back("26y");
    csv.slotnames.push_back("27y");
    csv.slotnames.push_back("28y");
    csv.slotnames.push_back("29y");
    csv.slotnames.push_back("30y");
    csv.slotnames.push_back("31y");
    csv.slotnames.push_back("32y");
    csv.slotnames.push_back("33y");
    csv.slotnames.push_back("34y");


    csv.slotnames.push_back("00s");
    csv.slotnames.push_back("01s");
    csv.slotnames.push_back("02s");
    csv.slotnames.push_back("03s");
    csv.slotnames.push_back("04s");
    csv.slotnames.push_back("05s");
    csv.slotnames.push_back("06s");
    csv.slotnames.push_back("07s");
    csv.slotnames.push_back("08s");
    csv.slotnames.push_back("09s");
    csv.slotnames.push_back("10s");
    csv.slotnames.push_back("11s");
    csv.slotnames.push_back("12s");
    csv.slotnames.push_back("13s");
    csv.slotnames.push_back("14s");
    csv.slotnames.push_back("15s");
    csv.slotnames.push_back("16s");
    csv.slotnames.push_back("17s");
    csv.slotnames.push_back("18s");
    csv.slotnames.push_back("19s");
    csv.slotnames.push_back("20s");
    csv.slotnames.push_back("21s");
    csv.slotnames.push_back("22s");
    csv.slotnames.push_back("23s");
    csv.slotnames.push_back("24s");
    csv.slotnames.push_back("25s");
    csv.slotnames.push_back("26s");
    csv.slotnames.push_back("27s");
    csv.slotnames.push_back("28s");
    csv.slotnames.push_back("29s");
    csv.slotnames.push_back("30s");
    csv.slotnames.push_back("31s");
    csv.slotnames.push_back("32s");
    csv.slotnames.push_back("33s");
    csv.slotnames.push_back("34s");

    csv.slotnames.push_back("00t");
    csv.slotnames.push_back("01t");
    csv.slotnames.push_back("02t");
    csv.slotnames.push_back("03t");
    csv.slotnames.push_back("04t");
    csv.slotnames.push_back("05t");
    csv.slotnames.push_back("06t");
    csv.slotnames.push_back("07t");
    csv.slotnames.push_back("08t");
    csv.slotnames.push_back("09t");
    csv.slotnames.push_back("10t");
    csv.slotnames.push_back("11t");
    csv.slotnames.push_back("12t");
    csv.slotnames.push_back("13t");
    csv.slotnames.push_back("14t");
    csv.slotnames.push_back("15t");
    csv.slotnames.push_back("16t");
    csv.slotnames.push_back("17t");
    csv.slotnames.push_back("18t");
    csv.slotnames.push_back("19t");
    csv.slotnames.push_back("20t");
    csv.slotnames.push_back("21t");
    csv.slotnames.push_back("22t");
    csv.slotnames.push_back("23t");
    csv.slotnames.push_back("24t");
    csv.slotnames.push_back("25t");
    csv.slotnames.push_back("26t");
    csv.slotnames.push_back("27t");
    csv.slotnames.push_back("28t");
    csv.slotnames.push_back("29t");
    csv.slotnames.push_back("30t");
    csv.slotnames.push_back("31t");
    csv.slotnames.push_back("32t");
    csv.slotnames.push_back("33t");
    csv.slotnames.push_back("34t");

    for(const auto& r : m_valid_ranges){
      for(unsigned cb_id = r.start; cb_id < r.end; ++cb_id){
	// z... x... y... s... t...
	// 0  1   35 36  71 
	for(unsigned c = 0; c < CB_WIDTH * CB_HEIGHT; ++c){
	  csv.push(c, m_cb_ir[cb_id].corners[c].z);
	  csv.push(c + 1 * 35, m_cb_ir[cb_id].corners[c].x);
	  csv.push(c + 2 * 35, m_cb_ir[cb_id].corners[c].y);
	  csv.push(c + 3 * 35, m_cb_rgb[cb_id].corners[c].u);
	  csv.push(c + 4 * 35, m_cb_rgb[cb_id].corners[c].v);
	}

      }
    }
    const std::string filename(std::string(prefix) + ".txt");
    csv.save(filename.c_str());

  }


  void
  ChessboardSampling::computeQualityFromSpeedIRInRanges(const float pose_offset){

    for(const auto& r : m_valid_ranges){
      const float best_frametime = std::min(0.0, r.avg_frametime - 3.0 * r.sd_frametime);
      const float worst_frametime = r.avg_frametime + 3.0 * r.sd_frametime;

      m_cb_ir[r.start].valid = 0;
      m_cb_rgb[r.start].valid = 0;

      for(unsigned cb_id = r.start + 1; cb_id < r.end; ++cb_id){
	const float curr_frametime = m_cb_ir[cb_id].time - m_cb_ir[cb_id - 1].time;
	if(curr_frametime > worst_frametime){
	  m_cb_ir[cb_id].valid = 0;
	  m_cb_rgb[cb_id].valid = 0;
	}
	else{
	  const float quality = 1.0f - (  (std::max(best_frametime, curr_frametime) - best_frametime) / (worst_frametime - best_frametime) );

	  for(unsigned c = 0; c < CB_WIDTH * CB_HEIGHT; ++c){
	    m_cb_ir[cb_id].quality[c] = quality;
	    m_cb_rgb[cb_id].quality[c] = quality;
	  }


	}
      }
    }

  }

  void
  ChessboardSampling::detectCorruptedDepthInRanges(){

    for(const auto& r : m_valid_ranges){

      std::vector<double> plane_qualities;
      for(unsigned cb_id = r.start; cb_id < r.end; ++cb_id){

	std::vector<xyz> corners;
	for(unsigned c = 0; c < CB_WIDTH * CB_HEIGHT; ++c){

	  xyz corner(m_cb_ir[cb_id].corners[c]);
	  corner.z *= 100.0f;
#if 0
	  // for testing
	  if(cb_id % 30 == 0 && c == 23)
	    corner.z = 0;
#endif
	  corners.push_back(corner);
	}

	const auto pq = detectPlaneQuality(corners);
	//std::cout << "cb_id: " << cb_id << " -> " << pq << std::endl;
	plane_qualities.push_back(pq);
      }


      double mean;
      double sd;
      calcMeanSD(plane_qualities, mean, sd);

      for(unsigned cb_id = r.start; cb_id < r.end; ++cb_id){
	const float curr_quality = plane_qualities[cb_id];
	if (curr_quality < (mean - 3.0 * sd)){
	  
	  std::cout << "found corrupted depth buffer in cb_id: " << cb_id << " -> plane quality: " << curr_quality << std::endl;
	  m_cb_ir[cb_id].valid = 0;
	  m_cb_rgb[cb_id].valid = 0;
	}
      }


    }

    

  }

  void
  ChessboardSampling::filterSamples(const float pose_offset){
    std::cerr << "ChessboardSampling::filterSamples -> begin" << std::endl;

    // 0. location where no corners where detected are already invalid


    // 1. gather valid ranges to detect flipps
    gatherValidRanges();
    detectFlips(); // better, more generic detectFlipsInRanges!


    // 1.5 detectCorruptedDepthInRanges
    gatherValidRanges();
    detectCorruptedDepthInRanges();


    // 2. gather valid ranges to detect time jumps
    gatherValidRanges();
    calcStatsInRanges();
    detectTimeJumpsInRanges();

   
    gatherValidRanges();
    calcStatsInRanges();
    
    // 3. apply OEFilter on ranges
    //oneEuroFilterInRanges();


#if 1
    gatherCornerTracesInRanges("corner_traces");
    exit(0);
#endif

    // 4. compute quality based on speed on range
    computeQualityFromSpeedIRInRanges(pose_offset);


    // filterCornerQualityInRanges();


    gatherValidRanges();
    calcStatsInRanges();

    for(auto& r : m_valid_ranges){
      std::cout << r << std::endl;
    }

    std::cerr << "ChessboardSampling::filterSamples -> end" << std::endl;

  }


  void
  ChessboardSampling::detectFlips(){

    for(unsigned cb_id = 0; cb_id < m_cb_ir.size(); ++cb_id){
      ChessboardViewRGB& cb_rgb = m_cb_rgb[cb_id];
      ChessboardViewIR& cb_ir  = m_cb_ir[cb_id];

      // if one of both is not valid both will be not valid
      cb_rgb.valid = cb_ir.valid && cb_rgb.valid;
      cb_ir.valid = cb_rgb.valid;

      if(cb_ir.valid){

	bool rgb_orientation;
	bool ir_orientation;
	{
	  glm::vec2 a(cb_rgb.corners[CB_WIDTH - 1].u - cb_rgb.corners[0].u, cb_rgb.corners[CB_WIDTH - 1].v - cb_rgb.corners[0].v);
	  glm::vec2 b(cb_rgb.corners[(CB_WIDTH * CB_HEIGHT) - CB_WIDTH].u - cb_rgb.corners[0].u, cb_rgb.corners[(CB_WIDTH * CB_HEIGHT) - CB_WIDTH].v - cb_rgb.corners[0].v);
	  rgb_orientation = a.x > 0.0 && b.y > 0.0;
	}
	{
	  glm::vec2 a(cb_ir.corners[CB_WIDTH - 1].x - cb_ir.corners[0].x, cb_ir.corners[CB_WIDTH - 1].y - cb_ir.corners[0].y);
	  glm::vec2 b(cb_ir.corners[(CB_WIDTH * CB_HEIGHT) - CB_WIDTH].x - cb_ir.corners[0].y, cb_ir.corners[(CB_WIDTH * CB_HEIGHT) - CB_WIDTH].y - cb_ir.corners[0].y);
	  ir_orientation = a.x > 0.0 && b.y > 0.0;
	}

	if( !(rgb_orientation && ir_orientation) ){
	  cb_rgb.valid = 0;
	  cb_ir.valid = 0;

	  std::cout << "ChessboardSampling::detectFlips() -> detected flip at " << cb_id << std::endl;

	}
      }
    }

  }



  double 
  ChessboardSampling::searchStartIR() const{
    return m_cb_ir[10 < m_cb_ir.size() ? 10 : 0].time;
  }


float
ChessboardSampling::computeAVGRGBFrequency(){
  size_t num_frames = 0;
  double freq = 0.0;
  for(unsigned i = 1; i < m_cb_rgb.size(); ++i){
    ++num_frames;
    freq += (1000.0 / (1000.0 * (m_cb_rgb[i].time - m_cb_rgb[i-1].time)));
  }
  return freq / num_frames;
}

float
ChessboardSampling::computeAVGIRFrequency(){
  size_t num_frames = 0;
  double freq = 0.0;
  for(unsigned i = 1; i < m_cb_ir.size(); ++i){
    ++num_frames;
    freq += (1000.0 / (1000.0 * (m_cb_ir[i].time - m_cb_ir[i-1].time)));
  }
  return freq / num_frames;
}



void
ChessboardSampling::calcStatsInRanges(){

  for(auto& r : m_valid_ranges){
    std::vector<float> frame_times;
    for(unsigned cb_id = r.start + 1; cb_id < r.end; ++cb_id){
      frame_times.push_back(m_cb_ir[cb_id].time - m_cb_ir[cb_id - 1].time);
    }
    calcMeanSDMaxMedian(frame_times, r.avg_frametime, r.sd_frametime, r.max_frametime, r.median_frametime);
  }

}

void
ChessboardSampling::gatherValidRanges(){
#define MIN_RANGE_SIZE 10

  // input is always m_cb_rgb and m_cb_ir

  // ouptut is always m_valid_ranges;

  m_valid_ranges.clear();

  ChessboardRange range_curr;
  range_curr.start = 0;
  range_curr.end = 0;
  bool valid = true;
  for(unsigned i = 0; i < m_cb_ir.size(); ++i){
    
    if(valid){
      if(m_cb_ir[i].valid){
	++range_curr.end;
      }
      else{
	if((range_curr.end - range_curr.start) > MIN_RANGE_SIZE){
	  m_valid_ranges.push_back(range_curr);
	}
	valid = false;
      }
    }
    else{
      if(m_cb_ir[i].valid){
	range_curr.start = i;
	range_curr.end = i + 1;
	valid = true;
      }
    }

  }

  if((range_curr.end - range_curr.start) > MIN_RANGE_SIZE){
    m_valid_ranges.push_back(range_curr);
  }

  std::cout << "ChessboardSampling::gatherValidRanges() -> valid ranges: " << m_valid_ranges.size() << std::endl;



}
