#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <NearestNeighbourSearch.hpp>

#include <squish.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <fstream>
#include <sstream>


namespace{

  // bilateral filter
  ////////////////////////////////////////////////////////////////////
  int kernel_size = 6; // in pixel
  int kernel_end = kernel_size + 1;
  float dist_space_max_inv = 1.0/float(kernel_size);


  float computeGaussSpace(float dist_space){
    float gauss_coord = dist_space * dist_space_max_inv;
    return 1.0 - gauss_coord;
  }

  float dist_range_max = 0.05; // in meter
  float dist_range_max_inv = 1.0/dist_range_max;

  float computeGaussRange(float dist_range){
    float gauss_coord = std::min(dist_range, dist_range_max) * dist_range_max_inv;
    return 1.0 - gauss_coord;
  }

  bool is_outside(const float d, const unsigned s_num, const std::vector<CalibVolume*> cvs){
    return (d < cvs[s_num]->min_d) || (d > cvs[s_num]->max_d);
  }

  float look_up_depth(const int x, const int y, const unsigned s_num, const RGBDSensor& sensor){

    if( (x < 0) ||
	(x > (sensor.config.size_d.x - 1)) ||
	(y < 0) ||
	(x > (sensor.config.size_d.y - 1))
	){
      return 0.0;
    }
    const unsigned d_idx = y* sensor.config.size_d.x + x;
    return s_num == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[s_num - 1][d_idx];
  }



  float bilateral_filter(const int x, const int y, const RGBDSensor& sensor, const unsigned s_num, const std::vector<CalibVolume*> cvs){
    float filtered_depth = 0.0;

    float depth = look_up_depth(x, y, s_num, sensor);
    if(is_outside(depth, s_num, cvs)){
      return 0.0;
    }


    // the valid range scales with depth
    float max_depth = 4.5; // Kinect V2
    float d_dmax = depth/max_depth;
    dist_range_max = 0.35 * d_dmax; // threshold around 
    dist_range_max_inv = 1.0/dist_range_max;

    float depth_bf = 0.0;

    float w = 0.0;
    float w_range = 0.0;
    float border_samples = 0.0;
    float num_samples = 0.0;
    
    for(int y_s = -kernel_size; y_s < kernel_end; ++y_s){
      for(int x_s = -kernel_size; x_s < kernel_end; ++x_s){
	num_samples += 1.0;
		
	const float depth_s = look_up_depth(x + x_s, y + y_s, s_num, sensor);

	const float depth_range = std::abs(depth_s - depth);
	if(is_outside(depth_s, s_num, cvs) || (depth_range > dist_range_max)){
	  border_samples += 1.0;
	  continue;
	}
	
	float gauss_space = computeGaussSpace(glm::length(glm::vec2(x_s,y_s)));
	float gauss_range = computeGaussRange(depth_range);
	float w_s = gauss_space * gauss_range;
	depth_bf += w_s * depth_s;
	w += w_s;
	w_range += gauss_range;
      }
    }
    
    const float lateral_quality  = 1.0 - border_samples/num_samples;
    
    if(w > 0.0)
      filtered_depth = depth_bf/w;
    else
      filtered_depth = 0.0;
    
    
    if(w_range < (num_samples * 0.65)){
      filtered_depth = 0.0;
    }

    return filtered_depth;

  }




  template <class T>
  inline std::string
  toStringP(T value, unsigned p)
  {
    std::ostringstream stream;
    stream << std::setw(p) << std::setfill('0') << value;
    return stream.str();
  }


  float calcAvgDist(const std::vector<nniSample>& neighbours, const nniSample& s){
    float avd = 0.0;
    for(const auto& n : neighbours){
      const float dist = glm::length(glm::vec3(s.s_pos.x - n.s_pos.x,s.s_pos.y - n.s_pos.y,s.s_pos.z - n.s_pos.z));
      avd += dist;
    }
    avd /= neighbours.size();
    return avd;
  }


  glm::vec3 bbx_min = glm::vec3(-1.2, -0.05, -1.2);
  glm::vec3 bbx_max = glm::vec3( 1.2, 2.4,  1.2);
  
  bool clip(const glm::vec3& p){
    if(p.x < bbx_min.x ||
       p.y < bbx_min.y ||
       p.z < bbx_min.z ||
       p.x > bbx_max.x ||
       p.y > bbx_max.y ||
       p.z > bbx_max.z){
      return true;
    }
    return false;
  }

  float filter_pass_1_max_avg_dist_in_meter = 0.025;
  float filter_pass_2_sd_fac = 1.0;
  unsigned filter_pass_1_k = 50;
  unsigned filter_pass_2_k = 50;

  void filterPerThread(NearestNeighbourSearch* nns, std::vector<nniSample>* nnisamples, std::vector<std::vector<nniSample> >* results, const unsigned tid, const unsigned num_threads){

    
    for(unsigned sid = tid; sid < nnisamples->size(); sid += num_threads){
      
      nniSample s = (*nnisamples)[sid];
      
      if(filter_pass_1_k > 2){      
	std::vector<nniSample> neighbours = nns->search(s,filter_pass_1_k);
	if(neighbours.empty()){
	  continue;
	}
	
	const float avd = calcAvgDist(neighbours, s);
	if(avd > filter_pass_1_max_avg_dist_in_meter){
	  continue;
	}
      }

      if(filter_pass_2_k > 2){
	std::vector<nniSample> neighbours = nns->search(s,filter_pass_2_k);
	const float avd = calcAvgDist(neighbours, s);
	std::vector<float> dists;
	for(const auto& n : neighbours){
	  std::vector<nniSample> local_neighbours = nns->search(n,filter_pass_2_k);
	  if(!local_neighbours.empty()){
	    dists.push_back(calcAvgDist(local_neighbours, n));
	  }
	}
	double mean;
	double sd;
	calcMeanSD(dists, mean, sd);
	if((avd - mean) > filter_pass_2_sd_fac * sd){
	  continue;
	}
      }
      (*results)[tid].push_back(s);
    }

  }

}



int main(int argc, char* argv[]){


  unsigned num_threads = 16;
  bool rgb_is_compressed = false;
  std::string stream_filename;
  CMDParser p("basefilename_cv .... basefilename_for_output");
  p.addOpt("s",1,"stream_filename", "specify the stream filename which should be converted");
  p.addOpt("n",1,"num_threads", "specify how many threads should be used, default 16");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed support for rgb stream, default: false (not compressed)");

  p.addOpt("p1d",1,"filter_pass_1_max_avg_dist_in_meter", "filter pass 1 skips points which have an average distance of more than this to it k neighbors, default 0.025");
  p.addOpt("p1k",1,"filter_pass_1_k", "filter pass 1 number of neighbors, default 50");
  p.addOpt("p2s",1,"filter_pass_2_sd_fac", "filter pass 2, specify how many times a point's distance should be above (values higher than 1.0) / below (values smaller than 1.0) is allowed to be compared to standard deviation of its k neighbors, default 1.0");
  p.addOpt("p2k",1,"filter_pass_2_k", "filter pass 2 number of neighbors (the higher the more to process), default 50");

  p.addOpt("bbx",6,"bounding_box", "specify the bounding box x_min y_min z_min x_max y_max z_max in meters, default -1.2 -0.05 -1.2 1.2 2.4 1.2");

  p.addOpt("b",1,"bil_filter_depth_kernel", "specify the kernel size of the bilateral depth filter, e.g. -b 6, default 0 (no bilateral filter ist applied)");

  p.addOpt("f",1,"frames", "specify how many frames should be processed at maximum, e.g. -f 1, default 0 (all frames in the stream will be processed)");

  p.init(argc,argv);


  if(p.isOptSet("bbx")){
    bbx_min = glm::vec3(p.getOptsFloat("bbx")[0], p.getOptsFloat("bbx")[1], p.getOptsFloat("bbx")[2]);
    bbx_max = glm::vec3(p.getOptsFloat("bbx")[3], p.getOptsFloat("bbx")[4], p.getOptsFloat("bbx")[5]);
    std::cout << "setting bounding box to min: " << bbx_min << " -> max: " << bbx_max << std::endl;
  }


  if(p.isOptSet("p1d")){
    filter_pass_1_max_avg_dist_in_meter = p.getOptsFloat("p1d")[0];
    std::cout << "setting filter_pass_1_max_avg_dist_in_meter to " << filter_pass_1_max_avg_dist_in_meter << std::endl;
  }
  if(p.isOptSet("p1k")){
    filter_pass_1_k = p.getOptsInt("p1k")[0];
    std::cout << "setting filter_pass_1_k to " << filter_pass_1_k << std::endl;
  }
  if(p.isOptSet("p2s")){
    filter_pass_2_sd_fac = p.getOptsFloat("p2s")[0];
    std::cout << "setting filter_pass_2_sd_fac to " << filter_pass_2_sd_fac << std::endl;
  }
  if(p.isOptSet("p2k")){
    filter_pass_2_k = p.getOptsInt("p2k")[0];
    std::cout << "setting filter_pass_2_k to " << filter_pass_2_k << std::endl;
  }
  


  if(p.isOptSet("s")){
    stream_filename = p.getOptsString("s")[0];
  }
  else{
    std::cerr << "ERROR, please specify stream filename with flag -s, see " << argv[0] << " -h for help" << std::endl;
    return 0;
  }

  if(p.isOptSet("n")){
    num_threads = p.getOptsInt("n")[0];
  }

  if(p.isOptSet("c")){
    rgb_is_compressed = true;
  }

  bool using_bf = false;
  if(p.isOptSet("b")){
    kernel_size = std::max(0, p.getOptsInt("b")[0]);
    kernel_end = kernel_size + 1;
    dist_space_max_inv = 1.0/float(kernel_size);
    using_bf = true;
    std::cout << "performing bilateral filtering with kernel size of: " << kernel_size << std::endl;
  }



  const unsigned num_streams(p.getArgs().size() - 1);
  const std::string basefilename_for_output = p.getArgs()[num_streams];
	
  std::vector<CalibVolume*> cvs;
  for(unsigned i = 0; i < num_streams; ++i){
    std::string basefilename = p.getArgs()[i];
    std::string filename_xyz(basefilename + "_xyz");
    std::string filename_uv(basefilename + "_uv");
    cvs.push_back(new CalibVolume(filename_xyz.c_str(), filename_uv.c_str()));
  }

	
  RGBDConfig cfg;
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);
  RGBDSensor sensor(cfg, num_streams - 1);	

  unsigned char* tmp_rgb = 0;
  unsigned char* tmp_rgba = 0;
  const unsigned colorsize_tmp = cfg.size_rgb.x * cfg.size_rgb.y * 3;
  const unsigned colorsize_tmpa = cfg.size_rgb.x * cfg.size_rgb.y * 4;
  if(rgb_is_compressed){
    tmp_rgb = new unsigned char [colorsize_tmp];
    tmp_rgba = new unsigned char [colorsize_tmpa];
  }	
  const unsigned colorsize = rgb_is_compressed ? 691200 : cfg.size_rgb.x * cfg.size_rgb.y * 3;
  const unsigned depthsize = cfg.size_d.x * cfg.size_d.y * sizeof(float);

  FileBuffer fb(stream_filename.c_str());
  if(!fb.open("r")){
    std::cerr << "ERROR, while opening " << stream_filename << " exiting..." << std::endl;
    return 1;
  }

  unsigned num_frames = fb.calcNumFrames(num_streams * (colorsize + depthsize));
  if(p.isOptSet("f")){
    num_frames = std::min(std::max(0u, (unsigned) p.getOptsInt("f")[0]), num_frames);
    std::cout << "processing " << num_frames << " frames of the stream" << std::endl;
  }
  double curr_frame_time = 0.0;

  unsigned frame_num = 0;
  while(frame_num < num_frames){
    ++frame_num;

    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      fb.read((unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]), colorsize);

      if(s_num == 0){
	memcpy((char*) &curr_frame_time, (const char*) sensor.frame_rgb, sizeof(double));
	std::cout << "curr_frame_time: " << curr_frame_time << std::endl;
      }

      if(rgb_is_compressed){
         // uncompress to rgb_tmp from (unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]) to tmp_rgba
         squish::DecompressImage (tmp_rgba, cfg.size_rgb.x, cfg.size_rgb.y,
                                  (unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]), squish::kDxt1);
         // copy back rgbsensor
         unsigned buffida = 0;
         unsigned buffid = 0;
         for(unsigned y = 0; y < cfg.size_rgb.y; ++y){
	   for(unsigned x = 0; x < cfg.size_rgb.x; ++x){
	     tmp_rgb[buffid++] = tmp_rgba[buffida++];
	     tmp_rgb[buffid++] = tmp_rgba[buffida++];
	     tmp_rgb[buffid++] = tmp_rgba[buffida++];
	     buffida++;
           }
         }
         memcpy((unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]), tmp_rgb, colorsize_tmp);
      }


      fb.read((unsigned char*) (s_num == 0 ? sensor.frame_d : sensor.slave_frames_d[s_num - 1]), depthsize);
    }



    std::vector<nniSample> nnisamples;
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      // do 3D reconstruction for each depth pixel
      for(unsigned y = 0; y < sensor.config.size_d.y; ++y){
	for(unsigned x = 0; x < (sensor.config.size_d.x - 3); ++x){

	  float d = 0.0;
	  if(!using_bf){
	    const unsigned d_idx = y* sensor.config.size_d.x + x;
	    d = s_num == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[s_num - 1][d_idx];
	  }
	  else{
	    d = bilateral_filter(x, y, sensor, s_num, cvs);
	  }

	  if(d < cvs[s_num]->min_d || d > cvs[s_num]->max_d){
	    continue;
	  }

	  glm::vec3 pos3D;
	  glm::vec2 pos2D_rgb;
	  
	  pos3D = cvs[s_num]->lookupPos3D( x * 1.0/sensor.config.size_d.x,
					   y * 1.0/sensor.config.size_d.y, d);

	  if(clip(pos3D)){
	    continue;
	  }

	  nniSample nnis;
	  nnis.s_pos.x = pos3D.x;
	  nnis.s_pos.y = pos3D.y;
	  nnis.s_pos.z = pos3D.z;		
		
	  glm::vec2 pos2D_rgb_norm = cvs[s_num]->lookupPos2D_normalized( x * 1.0/sensor.config.size_d.x, 
									 y * 1.0/sensor.config.size_d.y, d);
	  pos2D_rgb = glm::vec2(pos2D_rgb_norm.x * sensor.config.size_rgb.x,
				pos2D_rgb_norm.y * sensor.config.size_rgb.y);
	  
	  glm::vec3 rgb = sensor.get_rgb_bilinear_normalized(pos2D_rgb, s_num);

          
	  nnis.s_pos_off.x = rgb.x;
	  nnis.s_pos_off.y = rgb.y;
	  nnis.s_pos_off.z = rgb.z;


          nnisamples.push_back(nnis);

	}
      }
    }
    
    // optionally compute normals for each nniSample here using ComputeNormalsKnobi.hpp/.cpp


    
    std::cout << "start building acceleration structure for filtering " << nnisamples.size() << " points..." << std::endl;
    NearestNeighbourSearch nns(nnisamples);
    std::vector<std::vector<nniSample> > results;
    for(unsigned tid = 0; tid < num_threads; ++tid){
      results.push_back(std::vector<nniSample>() );
    }

    std::cout << "start filtering frame " << frame_num << " using " << num_threads << " threads." << std::endl;



    boost::thread_group threadGroup;
    for (unsigned tid = 0; tid < num_threads; ++tid){
      threadGroup.create_thread(boost::bind(&filterPerThread, &nns, &nnisamples, &results, tid, num_threads));
    }
    threadGroup.join_all();


    const std::string pcfile_name(basefilename_for_output + "_" + toStringP(frame_num, 5 /*fill*/) + ".ply");
    std::ofstream pcfile(pcfile_name.c_str());
    std::cout << "start writing to file " << pcfile_name << " ..." << std::endl;
    
    size_t num_points = 0;
    for(unsigned tid = 0; tid < num_threads; ++tid){
      num_points += results[tid].size();
    }

    pcfile << "ply" << std::endl;
    pcfile << "format ascii 1.0" << std::endl;
    pcfile << "element vertex " << num_points << std::endl;
    pcfile << "property float x" << std::endl;
    pcfile << "property float y" << std::endl;
    pcfile << "property float z" << std::endl;
    //pcfile << "property float nx" << std::endl;
    //pcfile << "property float ny" << std::endl;
    //pcfile << "property float nz" << std::endl;
    pcfile << "property uchar red" << std::endl;
    pcfile << "property uchar green" << std::endl;
    pcfile << "property uchar blue" << std::endl;
    pcfile << "end_header" << std::endl;

    for(unsigned tid = 0; tid < num_threads; ++tid){
      for(const auto& s : results[tid]){

	       int red   = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.x * 255.0f));
	       int green = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.y * 255.0f));
	       int blue  = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.z * 255.0f));
	       pcfile << s.s_pos.x << " " << s.s_pos.y << " " << s.s_pos.z << " "
	       //<< "0.0" << " " << "1.0" << " " << "0.0" << " "
         << red << " "
	       << green << " "
	       << blue << std::endl;
	
      }
      
    }

    pcfile.close();

    const std::string tsfile_name(basefilename_for_output + "_" + toStringP(frame_num, 5 /*fill*/) + ".timestamp");
    std::ofstream tsfile(tsfile_name.c_str());
    tsfile << curr_frame_time << std::endl;
    tsfile.close();

    std::cout << frame_num
	      << " from "
	      << num_frames
	      << " processed and saved to: "
	      << pcfile_name << std::endl << std::endl;

  }
  
  return 0;
}
