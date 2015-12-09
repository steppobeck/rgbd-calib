#include "sweepsampler.hpp"

#include <rgbdsensor.hpp>
#include <calibvolume.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <fstream>

SweepSampler::SweepSampler(const Checkerboard* cb, const CalibVolume* cv, const RGBDConfig * cfg)
  : m_sps(),
    m_cb(cb),
    m_cv(cv),
    m_cfg(cfg)
{}

SweepSampler::~SweepSampler()
{}




void
SweepSampler::dumpSamplePoints(){

  for(const auto& s : m_sps){
    std::cout << s << std::endl;
  }
  
}


const std::vector<samplePoint>&
SweepSampler::getSamplePoints(){
  return m_sps;
}


void
SweepSampler::appendSamplesToFile(const char* filename){

  std::ofstream off(filename, std::ofstream::binary | std::ofstream::app | std::ofstream::out);
  for(const auto& s : m_sps){
    off.write((const char*) &s.depth, sizeof(float));
    off.write((const char*) &s.tex_color, sizeof(uv));
    off.write((const char*) &s.tex_depth, sizeof(uv));
    off.write((const char*) &s.pos_offset, sizeof(xyz));
    off.write((const char*) &s.tex_offset, sizeof(uv));
    off.write((const char*) glm::value_ptr(s.pos_real), sizeof(glm::vec3));
    off.write((const char*) &s.quality, sizeof(float));
  }
  off.close();
  std::cout << "SweepSampler:: appended to file: " << filename << " num samples: " << m_sps.size() << std::endl;
}



size_t
SweepSampler::extractSamples(ChessboardSampling* cbs, const float pose_offset, const float color_offset){

  cbs->filterSamples(pose_offset);

  exit(0);

  const std::vector<ChessboardViewIR>& cb_irs = cbs->getIRs();
  for(unsigned i = 0; i != cb_irs.size(); ++i){
    const double time = cb_irs[i].time;
    // interpolate color_chessboard 
    bool valid_rgb;
    ChessboardViewRGB cb_rgb_i = cbs->interpolateRGB(time + color_offset, valid_rgb);
    // interpolate pose
    bool valid_pose;
    glm::mat4 pose_i = cbs->interpolatePose(time + pose_offset, valid_pose);
    if(valid_rgb && valid_pose){
      // add 35 corners to samplesPoints
      addBoardSamples(pose_i, &cb_irs[i], &cb_rgb_i);
    }
  }

  return m_sps.size();
}


void
SweepSampler::addBoardSamples(const glm::mat4& cb_transform, const ChessboardViewIR* corners_ir, const ChessboardViewRGB* corners_rgb){


  const unsigned cv_width = m_cv->width;
  const unsigned cv_height = m_cv->height;
  const unsigned cv_depth = m_cv->depth;

  for(unsigned idx = 0; idx < (CB_WIDTH * CB_HEIGHT); ++idx){
      
    uv  c_c(corners_rgb->corners[idx]);
    xyz c_d(corners_ir->corners[idx]);
    uv  c_d_tex;
    c_d_tex.u = c_d.x;
    c_d_tex.v = c_d.y;

    samplePoint sp;
    if(0.0 == corners_ir->quality[idx]){
      std::cerr << "ERROR: corner quality == 0" << std::endl;
    }
    sp.quality = corners_ir->quality[idx];
    sp.tex_color = c_c;
    sp.tex_depth = c_d_tex;
    sp.depth = c_d.z;


    glm::vec4 pos_realH = (m_cb->pose_offset * cb_transform) * glm::vec4(m_cb->points_local[idx].x, m_cb->points_local[idx].y, m_cb->points_local[idx].z, 1.0f);
    sp.pos_real = glm::vec3(pos_realH.x, pos_realH.y, pos_realH.z);
    
    
    const float x = cv_width  *  ( sp.tex_depth.u)/ m_cfg->size_d.x;
    const float y = cv_height *  ( sp.tex_depth.v)/ m_cfg->size_d.y;
    const float z = cv_depth  *  ( sp.depth - m_cv->min_d)/(m_cv->max_d - m_cv->min_d);
    
    xyz pos = getTrilinear(m_cv->cv_xyz, cv_width, cv_height, cv_depth, x , y , z );
    uv  tex = getTrilinear(m_cv->cv_uv, cv_width, cv_height, cv_depth, x , y , z );
    
    sp.pos_offset.x = sp.pos_real[0] - pos.x;
    sp.pos_offset.y = sp.pos_real[1] - pos.y;
    sp.pos_offset.z = sp.pos_real[2] - pos.z;
    
    sp.tex_offset.u = sp.tex_color.u/m_cfg->size_rgb.x - tex.u;
    sp.tex_offset.v = sp.tex_color.v/m_cfg->size_rgb.y - tex.v;
    
    m_sps.push_back(sp);
    
    
  }


}

