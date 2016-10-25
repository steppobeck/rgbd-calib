#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <NearestNeighbourSearch.hpp>
#include <udpconnection.hpp>
#include <squish.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <fstream>
#include <sstream>


namespace{

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


// ./send_recording_as_point_cloud /mnt/pitoti/kinect_recordings/25.8b/23.cv /mnt/pitoti/kinect_recordings/25.8b/24.cv /mnt/pitoti/kinect_recordings/25.8b/25.cv /mnt/pitoti/kinect_recordings/25.8b/26.cv -s /mnt/pitoti/kinect_recordings/25.8b/session_2.stream -c -p1k 0 -p2k 0 -bbx -1.5 0.0 -1.5 1.5 2.4 1.5
int main(int argc, char* argv[]){


  unsigned num_threads = 16;
  bool rgb_is_compressed = false;
  std::string stream_filename;

  udpconnection* sender = 0;

  CMDParser p("basefilename_cv .... ");
  p.addOpt("s",1,"stream_filename", "specify the stream filename which should be converted");
  p.addOpt("n",1,"num_threads", "specify how many threads should be used, default 16");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed support for rgb stream, default: false (not compressed)");

  p.addOpt("p1d",1,"filter_pass_1_max_avg_dist_in_meter", "filter pass 1 skips points which have an average distance of more than this to it k neighbors, default 0.025");
  p.addOpt("p1k",1,"filter_pass_1_k", "filter pass 1 number of neighbors, default 50");
  p.addOpt("p2s",1,"filter_pass_2_sd_fac", "filter pass 2, specify how many times a point's distance should be above (values higher than 1.0) / below (values smaller than 1.0) is allowed to be compared to standard deviation of its k neighbors, default 1.0");
  p.addOpt("p2k",1,"filter_pass_2_k", "filter pass 2 number of neighbors (the higher the more to process) , default 50");

  p.addOpt("bbx",6,"bounding_box", "specify the bounding box x_min y_min z_min x_max y_max z_max in meters, default -1.2 -0.05 -1.2 1.2 2.4 1.2");

  p.addOpt("u",2,"udpconncetion", "specify hostname and port of receiving udpconnection (client)");

  p.init(argc,argv);

  if(p.isOptSet("u")){
    sender = new udpconnection(p.getOptsString("u")[0].c_str(), p.getOptsInt("u")[1]);
    sender->open();
  }
  else{
    p.showHelp();
    return 0;
  }

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

  const unsigned num_streams(p.getArgs().size());
  
	
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
  const unsigned num_frames = fb.calcNumFrames(num_streams * (colorsize + depthsize));
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
	for(unsigned x = 0; x < sensor.config.size_d.x; ++x){
	  const unsigned d_idx = y* sensor.config.size_d.x + x;
	  float d = s_num == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[s_num - 1][d_idx];
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

    // finished of 3D reconstruction

    // calculate number of points in points cloud



    size_t timestamp = curr_frame_time * 1000;
    std::cout << "timestamp: " << timestamp << std::endl;
    /*
      layout of udp packet:
      timestamp_8 packetNumber_4 voxelCount_4 x_2 y_2 z_2 r_1 g_1 b_1 i_1 ...
      8 4 4 2 2 2 1 1 1 1
      memcpy(&message[0], &timeStamp, 8);
      memcpy(&message[8], &packetNumber, 4);
      memcpy(&message[12], &voxelCount, 4);
    */
    const unsigned max_voxels_per_packet = 6000;
    const unsigned byte_per_voxel = 10;
    const unsigned byte_of_header = 16;
    const unsigned max_bufflen = max_voxels_per_packet * byte_per_voxel + byte_of_header;
    
    unsigned char buff[max_bufflen];
    const float voxelsize = 0.008;
    size_t byte_index = 0;

    for(unsigned tid = 0; tid < num_threads; ++tid){
      for(const auto& s : results[tid]){


	// fill header

	unsigned char red   = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.x * 255.0f));
	unsigned char green = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.y * 255.0f));
	unsigned char blue  = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.z * 255.0f));

	// voxelize s.s_pos to unsigned short
	// 1. subtract bbx_min

	// 2. divide s.s_pos by voxelsize

	// copy "voxel into buff"

	/*
	  pcfile << s.s_pos.x << " " << s.s_pos.y << " " << s.s_pos.z << " "
	  << red << " "
	  << green << " "
	  << blue << std::endl;
	*/
	
      }
      
    }


  }
  
  return 0;
}
