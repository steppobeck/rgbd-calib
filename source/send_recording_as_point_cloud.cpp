#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <DataTypes.hpp>
#include <NearestNeighbourSearch.hpp>
#include <udpconnection.hpp>
#include <squish.h>
#include <clock.hpp>
#include <timevalue.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <fstream>
#include <sstream>


namespace{


  class voxel{
  public:
    voxel()
      : xyz(),
	rgbi()
    {
      xyz[0] = 0;
      xyz[1] = 0;
      xyz[2] = 0;
      rgbi[0] = 0;
      rgbi[1] = 0;
      rgbi[2] = 0;
      rgbi[3] = 0;
    }
    unsigned short xyz[3];
    unsigned char rgbi[4];
  };

  std::ostream& operator<< (std::ostream& os, const voxel& v){
    os << "voxel: " << v.xyz[0] << ", " << v.xyz[1] << ", " << v.xyz[2] << " color: " << (int) v.rgbi[0] << ", " << (int) v.rgbi[1] << ", " << (int) v.rgbi[2];
    return os;
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

  float voxelsize_inv = 1.0f/0.008;
  unsigned volume_width  = (bbx_max[0] - bbx_min[0])*voxelsize_inv;
  unsigned volume_height = (bbx_max[1] - bbx_min[1])*voxelsize_inv;

}

void decomp_per_thread(const unsigned s_num, std::vector<unsigned char*>& tmp_rgba, std::vector<unsigned char*>& tmp_rgb, const RGBDConfig& cfg, RGBDSensor& sensor, const unsigned colorsize_tmp){
  squish::DecompressImage (tmp_rgba[s_num], cfg.size_rgb.x, cfg.size_rgb.y,
			   (unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]), squish::kDxt1);
  // copy back rgbsensor
  unsigned buffida = 0;
  unsigned buffid = 0;
  for(unsigned y = 0; y < cfg.size_rgb.y; ++y){
    for(unsigned x = 0; x < cfg.size_rgb.x; ++x){
      tmp_rgb[s_num][buffid++] = tmp_rgba[s_num][buffida++];
      tmp_rgb[s_num][buffid++] = tmp_rgba[s_num][buffida++];
      tmp_rgb[s_num][buffid++] = tmp_rgba[s_num][buffida++];
      buffida++;
    }
  }
  memcpy((unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]), tmp_rgb[s_num], colorsize_tmp);
}

void recon_per_thread(const unsigned tid, std::vector<CalibVolume*>& cvs, RGBDSensor& sensor, std::map<size_t, std::vector<voxel>>* recon_volume, const unsigned start_y, const unsigned end_y){
  // do 3D reconstruction for each depth pixel

  

  for(unsigned y = start_y; y < end_y; ++y){
    for(unsigned x = 0; x < (sensor.config.size_d.x - 3); ++x){

      const unsigned d_idx = y* sensor.config.size_d.x + x;
      float d = tid == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[tid - 1][d_idx];
      if(d < cvs[tid]->min_d || d > cvs[tid]->max_d){
	continue;
      }

      glm::vec3 pos3D;
      glm::vec2 pos2D_rgb;
      
      pos3D = cvs[tid]->lookupPos3D( x * 1.0/sensor.config.size_d.x,
				     y * 1.0/sensor.config.size_d.y, d);
      
      if(clip(pos3D)){
	continue;
      }


      glm::vec2 pos2D_rgb_norm = cvs[tid]->lookupPos2D_normalized( x * 1.0/sensor.config.size_d.x, 
								   y * 1.0/sensor.config.size_d.y, d);
      pos2D_rgb = glm::vec2(pos2D_rgb_norm.x * sensor.config.size_rgb.x,
			    pos2D_rgb_norm.y * sensor.config.size_rgb.y);
	  
      glm::vec3 rgb = sensor.get_rgb_bilinear_normalized(pos2D_rgb, tid);
      
      // discretize voxel here
      voxel vxl;
      vxl.xyz[0] = (unsigned short)((pos3D.x - bbx_min[0])*voxelsize_inv);
      vxl.xyz[1] = (unsigned short)((pos3D.y - bbx_min[1])*voxelsize_inv);
      vxl.xyz[2] = (unsigned short)((pos3D.z - bbx_min[2])*voxelsize_inv);
      // compute grid_loc of surface point
      const size_t grid_loc = vxl.xyz[0] + volume_width * vxl.xyz[1] + volume_width * volume_height * vxl.xyz[2];
      vxl.rgbi[0] = (unsigned char) std::max(0.0f , std::min(255.0f, rgb.x * 255.0f));
      vxl.rgbi[1] = (unsigned char) std::max(0.0f , std::min(255.0f, rgb.y * 255.0f));
      vxl.rgbi[2] = (unsigned char) std::max(0.0f , std::min(255.0f, rgb.z * 255.0f));
      (*recon_volume)[grid_loc].push_back(vxl);
    }
  }
}


// ./send_recording_as_point_cloud /mnt/pitoti/kinect_recordings/25.8b/23.cv /mnt/pitoti/kinect_recordings/25.8b/24.cv /mnt/pitoti/kinect_recordings/25.8b/25.cv /mnt/pitoti/kinect_recordings/25.8b/26.cv -s /mnt/pitoti/kinect_recordings/25.8b/session_1.stream -c -bbx -0.5 0.0 -0.5 0.5 2.0 0.5 -u 127.0.0.1 7002 -p1k 20 -p2k 3
int main(int argc, char* argv[]){

  
  
  bool rgb_is_compressed = false;
  std::string stream_filename;
  unsigned wait_ms = 0;
  udpconnection* sender = 0;

  CMDParser p("basefilename_cv .... ");
  p.addOpt("s",1,"stream_filename", "specify the stream filename which should be converted");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed support for rgb stream, default: false (not compressed)");

  p.addOpt("bbx",6,"bounding_box", "specify the bounding box x_min y_min z_min x_max y_max z_max in meters, default -1.2 -0.05 -1.2 1.2 2.4 1.2");

  p.addOpt("u",2,"udpconncetion", "specify hostname and port of receiving udpconnection (client)");
  
  p.addOpt("w",1,"wait", "specify how many milliseconds to wait between packets: default 0");
  p.addOpt("v",1,"voxelsize", "specify the voxel size in meter: default 0.008");


  p.init(argc,argv);

  if(p.isOptSet("u")){
    sender = new udpconnection;
    sender->open_sending_socket(p.getOptsString("u")[0].c_str(), p.getOptsInt("u")[1]);
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

  if(p.isOptSet("w")){
    wait_ms = p.getOptsInt("w")[0];
  }

  if(p.isOptSet("v")){
    voxelsize_inv = 1.0f/p.getOptsFloat("v")[0];
  }

  // compute volume dimensions
  volume_width  = (bbx_max[0] - bbx_min[0])*voxelsize_inv;
  volume_height = (bbx_max[1] - bbx_min[1])*voxelsize_inv;
  


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


  const unsigned colorsize = rgb_is_compressed ? 691200 : cfg.size_rgb.x * cfg.size_rgb.y * 3;
  const unsigned depthsize = cfg.size_d.x * cfg.size_d.y * sizeof(float);

  std::vector<unsigned char*> tmp_rgb;
  std::vector<unsigned char*> tmp_rgba;
  const unsigned colorsize_tmp = cfg.size_rgb.x * cfg.size_rgb.y * 3;
  const unsigned colorsize_tmpa = cfg.size_rgb.x * cfg.size_rgb.y * 4;
  if(rgb_is_compressed){
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      tmp_rgb.push_back(new unsigned char [colorsize_tmp]);
      tmp_rgba.push_back(new unsigned char [colorsize_tmpa]);
    }
  }


  FileBuffer fb(stream_filename.c_str());
  if(!fb.open("r")){
    std::cerr << "ERROR, while opening " << stream_filename << " exiting..." << std::endl;
    return 1;
  }
  const unsigned num_frames = fb.calcNumFrames(num_streams * (colorsize + depthsize));
  double curr_frame_time = 0.0;

  unsigned frame_num = 0;

  sensor::timevalue ts = sensor::clock::time();

  while(frame_num < num_frames){
    ++frame_num;
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! start of frame" << std::endl;
    sensor::timevalue ts_now = sensor::clock::time();
    std::cout << "frame time: " << (ts_now - ts).msec() << " ms" << std::endl;
    ts = ts_now;

    unsigned bytes_per_frame = 0;
    // start reading frames from file
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      fb.read((unsigned char*) (s_num == 0 ? sensor.frame_rgb : sensor.slave_frames_rgb[s_num - 1]), colorsize);
      bytes_per_frame += colorsize;
      if(s_num == 0){
	memcpy((char*) &curr_frame_time, (const char*) sensor.frame_rgb, sizeof(double));
	//std::cout << "curr_frame_time: " << curr_frame_time << std::endl;
      }
      fb.read((unsigned char*) (s_num == 0 ? sensor.frame_d : sensor.slave_frames_d[s_num - 1]), depthsize);
      bytes_per_frame += depthsize;
    }

    if(rgb_is_compressed){
      boost::thread_group threadGroup;
      for(unsigned s_num = 0; s_num < num_streams; ++s_num){
	threadGroup.create_thread([s_num, &tmp_rgba, &tmp_rgb, &cfg, &sensor, colorsize_tmp](){
				    decomp_per_thread(s_num, tmp_rgba, tmp_rgb, cfg, sensor, colorsize_tmp);
				  }
				  );
      }
      threadGroup.join_all();
    }


    sensor::timevalue ts_now_read = sensor::clock::time();
    std::cout << "frame time_read: " << (ts_now_read - ts_now).msec() << " ms" << std::endl;


    const unsigned num_threads = num_streams * 2;
    std::vector< std::map<size_t, std::vector<voxel>>> recon_volumes(num_threads);
    boost::thread_group threadGroup;
    unsigned tid = 0;
    for(unsigned s_num = 0; s_num != num_streams; ++s_num){
      threadGroup.create_thread([s_num, &cvs, &sensor, &recon_volumes, tid](){
				  recon_per_thread(s_num, cvs, sensor, &(recon_volumes[tid]), 0, sensor.config.size_d.y/2);
						   }
				);
      ++tid;
      threadGroup.create_thread([s_num, &cvs, &sensor, &recon_volumes, tid](){
				  recon_per_thread(s_num, cvs, sensor, &(recon_volumes[tid]), sensor.config.size_d.y/2, sensor.config.size_d.y);
						   }
				);
      ++tid;

    }
    threadGroup.join_all();


    // filter recon_volume
    std::vector<voxel> voxels;
    for(const auto& recon_volume : recon_volumes){
      for(const auto& vxls : recon_volume){
	voxels.push_back(vxls.second.front());
      }
    }
    unsigned voxel_count = voxels.size();
    std::cout << "voxel_count: " << voxel_count << std::endl;

    sensor::timevalue ts_now_recon = sensor::clock::time();
    std::cout << "frame time_recon: " << (ts_now_recon - ts_now_read).msec() << " ms" << std::endl;
    // finished of 3D reconstruction    

    

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
    
    size_t buff_index = 0;
    unsigned packet_number = 0;
    unsigned voxel_number = 0;
    unsigned bytes_sent_per_frame = 0;
    for(const auto& s : voxels){

      if(voxel_number == max_voxels_per_packet){
	std::cout << "sending packet number: " << packet_number
		  << " -> " << voxel_number
		  << " (" << buff_index << " bytes)" << std::endl;
	sender->send(buff, buff_index);
	bytes_sent_per_frame += buff_index;
	if(wait_ms)
	  sleep(sensor::timevalue::const_100_us * wait_ms);
	
	voxel_number = 0;
	buff_index = 0;
	++packet_number;
      }

      // fill header
      if(0 == voxel_number){
	memcpy(&buff[buff_index], &timestamp, sizeof(timestamp));
	buff_index += sizeof(timestamp);
	memcpy(&buff[buff_index], &packet_number, sizeof(packet_number));
	buff_index += sizeof(packet_number);
	memcpy(&buff[buff_index], &voxel_count, sizeof(voxel_count));
	buff_index += sizeof(voxel_count);
      }

      unsigned char r = s.rgbi[0];
      unsigned char g = s.rgbi[1];
      unsigned char b = s.rgbi[2];
      unsigned short x = s.xyz[0];
      unsigned short y = s.xyz[1];
      unsigned short z = s.xyz[2];

      // copy "voxel into buff"
      memcpy(&buff[buff_index], &x, sizeof(x));
      buff_index += sizeof(x);
      memcpy(&buff[buff_index], &y, sizeof(y));
      buff_index += sizeof(y);
      memcpy(&buff[buff_index], &z, sizeof(z));
      buff_index += sizeof(z);
      
      memcpy(&buff[buff_index], &r, sizeof(r));
      buff_index += sizeof(r);
      memcpy(&buff[buff_index], &g, sizeof(g));
      buff_index += sizeof(g);
      memcpy(&buff[buff_index], &b, sizeof(b));
      buff_index += sizeof(b);
      
      unsigned char joint_id = 0; // not used but needed by protocol
      memcpy(&buff[buff_index], &joint_id, sizeof(joint_id));
      buff_index += sizeof(joint_id);
      
      
      ++voxel_number;
    }

    if(voxel_number > 0){
      std::cout << "sending packer number: " << packet_number
		<< " -> " << voxel_number
		<< " (" << buff_index << " bytes)" << std::endl;
      sender->send(buff, buff_index);
      bytes_sent_per_frame += buff_index;
    }

    sensor::timevalue ts_now_send = sensor::clock::time();
    std::cout << "frame time_send: " << (ts_now_send - ts_now_recon).msec() << " ms" << std::endl;
    // finished sending
    
    // print some additional stats
    std::cout << "kbytes_sent_per_frame (kbytes_per_frame): " << bytes_sent_per_frame/1024 << " (" << bytes_per_frame/1024 << ") -> " << bytes_sent_per_frame * 100.0f/bytes_per_frame << "%" << std::endl;

    if(frame_num == num_frames){
      frame_num = 0;
      fb.rewindFile();
    }


  }


  if(rgb_is_compressed){
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      delete [] tmp_rgb[s_num];
      delete [] tmp_rgba[s_num];
    }
  }

  
  return 0;
}
