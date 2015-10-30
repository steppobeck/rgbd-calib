#include "stablesampler.hpp"
#include <rgbdsensor.hpp>
#include <calibvolume.hpp>
#include <ARTListener.hpp>
#include <OpenCVChessboardCornerDetector.hpp>

#include <PoseTracker.hpp>
#include <SampleFilter.hpp>


#include <unistd.h>

StableSampler::StableSampler(RGBDSensor* sensor, CalibVolume* cv, unsigned art_port, unsigned art_target_id, Checkerboard* cb)
  : m_sensor(sensor),
    m_cv(cv),
    m_artl(new ARTListener),
    m_art_target_id(art_target_id),
    m_cd_c(),
    m_cd_i(),
    m_sps(),
    m_errors(),
    m_cb(cb)
{
  m_artl->open(art_port);
  m_cd_c = new OpenCVChessboardCornerDetector(m_sensor->config.size_rgb.x,
					      m_sensor->config.size_rgb.y,
					      8 /*bits per channel*/,
					      3 /*num channels*/,
					      CB_WIDTH, CB_HEIGHT);


  m_cd_i = new OpenCVChessboardCornerDetector(m_sensor->config.size_d.x,
					      m_sensor->config.size_d.y,
					      8 /*bits per channel*/,
					      1,
					      CB_WIDTH, CB_HEIGHT);
}


void
StableSampler::sampleBoardLocation(){

  // sample until filtering is finished
  const unsigned num_frames_filter = 30;

  const unsigned pixelcountc = m_sensor->config.size_rgb.x * m_sensor->config.size_rgb.y;
  const unsigned pixelcount = m_sensor->config.size_d.x * m_sensor->config.size_d.y;
  const unsigned colorsize = pixelcountc * 3 * sizeof(unsigned char);
  const unsigned irsize = pixelcount * sizeof(unsigned char);


  unsigned num_frames_token = 0;
  SampleFilter sfilt(CB_WIDTH * CB_HEIGHT);
  PoseTracker pt;

  while(num_frames_token < num_frames_filter){
    m_sensor->recv(true);
    m_artl->listen();
    sleep(sensor::timevalue::const_050_ms);
    glm::mat4 cb_transform(m_artl->getMatrices()[m_art_target_id]);
    //std::cerr << cb_transform << std::endl;
    
    bool is_stable(pt.isStable(cb_transform, 0.0005, 30)); // LCD side
    std::cerr << "is_stable: " << (int) is_stable << std::endl;
    if(!is_stable){
      num_frames_token = 0;
      sfilt.clear();
      continue;
    }

    unsigned char* color_buffer = m_sensor->frame_rgb;
    float* depth_buffer = m_sensor->frame_d;
    unsigned char* ir_buffer = m_sensor->frame_ir;

    bool found_color_corners = m_cd_c->process((unsigned char*) color_buffer, colorsize, true);
    bool found_ir_corners = m_cd_i->process((unsigned char*) ir_buffer, irsize, true);

    std::vector<samplePoint> sps_tmp;
    if(found_color_corners && found_ir_corners && (m_cd_i->corners.size() == m_cd_c->corners.size())
       && m_cd_i->corners.size() == (CB_WIDTH * CB_HEIGHT)){

      // print depth values at corners in depth buffer;
      const std::vector<uv>& corners_depth = m_cd_i->corners;
      const std::vector<uv>& corners_color = m_cd_c->corners;

      std::cerr << "--------------------- FOUND corners -------------------: " << corners_depth.size() << std::endl;
      
      for(unsigned idx = 0; idx < corners_depth.size(); ++idx){
	uv c_d(corners_depth[idx]);
	uv c_c(corners_color[idx]);
	//std::cerr << idx << " " << c_d.u << " " << c_d.v << " " << getBilinear(depth_buffer, m_calibs[i]->getWidth(), m_calibs[i]->getHeight(), c_d.u, c_d.v) << std::endl;
	samplePoint sp;
	    
	sp.tex_color = c_c;
	sp.tex_depth = c_d;
	sp.depth = getBilinear(depth_buffer, m_sensor->config.size_d.x , m_sensor->config.size_d.y, c_d.u, c_d.v);

	glm::vec4 pos_realH = (m_cb->pose_offset * cb_transform) * glm::vec4(m_cb->points_local[idx].x, m_cb->points_local[idx].y, m_cb->points_local[idx].z, 1.0f);

	sp.pos_real = glm::vec3(pos_realH.x, pos_realH.y, pos_realH.z);

	sps_tmp.push_back(sp);
	
	//std::cerr << sp << std::endl;
	
      }
      
    }

    if(sps_tmp.size() != (CB_WIDTH * CB_HEIGHT)){
      std::cerr << "not enough samples found in frame...only: " << sps_tmp.size() << " not adding to filter" << std::endl;
      continue;
    }

    sfilt.addSamples(sps_tmp);
    ++num_frames_token;
    std::cerr << "samples frames to add: " << num_frames_filter -  num_frames_token << std::endl;    
  }


  std::vector<samplePoint> filtered_samples =  sfilt.getFiltered();
  std::cerr << "sampling done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!111" << std::endl;
    
	
  for(unsigned i = 0; i < filtered_samples.size(); ++i){
    const unsigned cv_width = m_cv->width;
    const unsigned cv_height = m_cv->height;
    const unsigned cv_depth = m_cv->depth;

    const float x = cv_width  *  ( filtered_samples[i].tex_depth.u) / m_sensor->config.size_d.x;
    const float y = cv_height *  ( filtered_samples[i].tex_depth.v)/ m_sensor->config.size_d.y;
    const float z = cv_depth  *  ( filtered_samples[i].depth - m_cv->min_d)/(m_cv->max_d - m_cv->min_d);

    xyz pos = getTrilinear(m_cv->cv_xyz, cv_width, cv_height, cv_depth, x , y , z );
    uv  tex = getTrilinear(m_cv->cv_uv,  cv_width, cv_height, cv_depth, x , y , z );


    filtered_samples[i].pos_offset.x = filtered_samples[i].pos_real[0] - pos.x;
    filtered_samples[i].pos_offset.y = filtered_samples[i].pos_real[1] - pos.y;
    filtered_samples[i].pos_offset.z = filtered_samples[i].pos_real[2] - pos.z;
      
    filtered_samples[i].tex_offset.u = filtered_samples[i].tex_color.u/m_sensor->config.size_rgb.x - tex.u;
    filtered_samples[i].tex_offset.v = filtered_samples[i].tex_color.v/m_sensor->config.size_rgb.y - tex.v;

    uv err;	  
    err.u = glm::length(glm::vec3(filtered_samples[i].pos_offset.x,filtered_samples[i].pos_offset.y,filtered_samples[i].pos_offset.z));
    err.v = glm::length(glm::vec3(filtered_samples[i].tex_offset.u * m_sensor->config.size_rgb.x,
				  filtered_samples[i].tex_offset.v * m_sensor->config.size_rgb.y,0.0));
    
    
    m_sps.push_back(filtered_samples[i]);
    m_errors.push_back(err);
      
  }


  // play sound to notice user and wait 5 seconds
  system("/usr/bin/aplay ../../../framework/click_x.wav");
  sleep(5);

}


const std::vector<samplePoint>&
StableSampler::getSamplePoints(){
  return m_sps;
}
