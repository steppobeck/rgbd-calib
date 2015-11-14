#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <NearestNeighbourSearch.hpp>

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

}

int main(int argc, char* argv[]){
	
  bool rgb_is_compressed = false;
  std::string stream_filename;
  CMDParser p("basefilename_cv .... basefilename_for_output");
  p.addOpt("s",1,"stream_filename", "specify the stream filename which should be converted");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed support for rgb stream (if set, color will be ignored), default: false (not compressed)");
  p.init(argc,argv);

  if(p.isOptSet("s")){
    stream_filename = p.getOptsString("s")[0];
  }
  else{
    std::cerr << "ERROR, please specify stream filename with flag -s, see " << argv[0] << " -h for help" << std::endl;
    return 0;
  }
  if(p.isOptSet("c")){
    rgb_is_compressed = true;
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
	
  const unsigned colorsize = rgb_is_compressed ? 691200 : cfg.size_rgb.x * cfg.size_rgb.y * 3;
  const unsigned depthsize = cfg.size_d.x * cfg.size_d.y * sizeof(float);

  FileBuffer fb(stream_filename.c_str());
  if(!fb.open("r")){
    std::cerr << "ERROR, while opening " << stream_filename << " exiting..." << std::endl;
    return 1;
  }
  const unsigned num_frames = fb.calcNumFrames(num_streams * (colorsize + depthsize));


  unsigned frame_num = 0;
  while(frame_num < num_frames){
    ++frame_num;

    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      fb.read((unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]), colorsize);
      fb.read((unsigned char*) (s_num == 0 ? sensor.frame_d : sensor.slave_frames_d[s_num - 1]), depthsize);
    }

    std::vector<nniSample> nnisamples;
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      // do 3D reconstruction for each depth pixel
      for(unsigned y = 0; y < sensor.config.size_d.y; ++y){
	for(unsigned x = 0; x < sensor.config.size_d.x; ++x){
	  const unsigned d_idx = y* sensor.config.size_d.x + x;
	  float d = s_num == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[s_num - 1][d_idx];
	  if(d < cvs[s_num]->min_d || d > cvs[s_num]->max_d)
	    continue;
	  
	  glm::vec3 pos3D;
	  glm::vec2 pos2D_rgb;
	  
	  pos3D = cvs[s_num]->lookupPos3D( x * 1.0/sensor.config.size_d.x,
					   y * 1.0/sensor.config.size_d.y, d);
		
	  // CLIP here against bounding box from DataTypes.hpp	
	  nniSample nnis;
	  nnis.s_pos.x = pos3D.x;
	  nnis.s_pos.y = pos3D.y;
	  nnis.s_pos.z = pos3D.z;		
		
	  if(!rgb_is_compressed){
	    glm::vec2 pos2D_rgb_norm = cvs[s_num]->lookupPos2D_normalized( x * 1.0/sensor.config.size_d.x, 
									   y * 1.0/sensor.config.size_d.y, d);
	    pos2D_rgb = glm::vec2(pos2D_rgb_norm.x * sensor.config.size_rgb.x,
				  pos2D_rgb_norm.y * sensor.config.size_rgb.y);
	  
	    glm::vec3 rgb = sensor.get_rgb_bilinear_normalized(pos2D_rgb, s_num);

          
	    nnis.s_pos_off.x = rgb.x;
	    nnis.s_pos_off.y = rgb.y;
	    nnis.s_pos_off.z = rgb.z;

	  }
	  else{
	    nnis.s_pos_off.x = 0.0;
	    nnis.s_pos_off.y = 0.0;
	    nnis.s_pos_off.z = 0.0;

	  }
          nnisamples.push_back(nnis);

	}
      }
    }
    
    const std::string pcfile_name(basefilename_for_output + "_" + toStringP(frame_num, 5 /*fill*/) + ".xyz");
    std::ofstream pcfile(pcfile_name.c_str());    
    std::cout << "start filtering " << frame_num << std::endl;
    NearestNeighbourSearch nns(nnisamples);
    for(auto s : nnisamples){
      // filter here
      const unsigned k = 50;
      std::vector<nniSample> neighbours = nns.search(s,k);
      if(neighbours.empty()){
	continue;
      }
      float avg_dist = 0.0;
      for(const auto& n : neighbours){
	const float dist = glm::length(glm::vec3(s.s_pos.x - n.s_pos.x,s.s_pos.y - n.s_pos.y,s.s_pos.z - n.s_pos.z));
	avg_dist += dist;
      }
      avg_dist /= k;
      if(avg_dist > 0.025){
	continue;
      }
      
      int red   = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.x * 255.0f));
      int green = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.y * 255.0f));
      int blue  = (int) std::max(0.0f , std::min(255.0f, s.s_pos_off.z * 255.0f));
      pcfile << s.s_pos.x << " " << s.s_pos.y << " " << s.s_pos.z << " "
	     << red << " "
	     << green << " "
	     << blue << std::endl;
  
    }
    pcfile.close();
    std::cout << frame_num
	      << " from "
	      << num_frames
	      << " processed and saved to: "
	      << pcfile_name << std::endl;

    return 0;

  }
  
  return 0;
}
