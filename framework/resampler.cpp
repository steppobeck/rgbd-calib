#include "resampler.hpp"

#include <calibvolume.hpp>
#include <NearestNeighbourSearch.hpp>
#include <calibrator.hpp>

#include <limits>
#include <random>
#include <map>
#include <fstream>


namespace{

#if 0
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
int main()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    // give "true" 1/4 of the time
    // give "false" 3/4 of the time
    std::bernoulli_distribution d(0.25);
 
    std::map<bool, int> hist;
    for(int n=0; n<10000; ++n) {
        ++hist[d(gen)];
    }
    for(auto p : hist) {
        std::cout << std::boolalpha << std::setw(5) << p.first
                  << ' ' << std::string(p.second/500, '*') << '\n';
    }
}
#endif


  class Coin{

  public:
    Coin(const float chance /*e.g. 0.25 -> 25% true , 75% false*/)
      : m_rd(),
	m_gen(m_rd()),
	m_d(chance)
    {}

    ~Coin()
    {}

    bool operator()(){
      return m_d(m_gen);
    }

  private:
    std::random_device m_rd;
    std::mt19937 m_gen;
    std::bernoulli_distribution m_d;

  };

#if 0
#include <random>
#include <iostream>
 
int main()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(1, 2);
    for (int n = 0; n < 10; ++n) {
        std::cout << dis(gen) << ' ';
    }
    std::cout << '\n';
}
#endif

  class RandRange{
  public:
    RandRange(const float a, const float b)
      : m_rd(),
	m_gen(m_rd()),
	m_dis(a, b)
    {}

    ~RandRange()
    {}

    float operator()(){
      return m_dis(m_gen);
    }

  private:
    std::random_device m_rd;
    std::mt19937 m_gen;
    std::uniform_real_distribution<float> m_dis;

  };

  nniSample operator * (float q, const nniSample& s){
    nniSample res;
    res.s_pos_cs  = q * s.s_pos_cs;
    res.s_pos     = q * s.s_pos;
    res.s_pos_off = q * s.s_pos_off;
    res.s_tex_off = q * s.s_tex_off;
    res.quality   = q * s.quality;
    
    return res;
  }

  nniSample operator + (const nniSample& a, const nniSample& b){
    nniSample res;
    res.s_pos_cs  = a.s_pos_cs + b.s_pos_cs;
    res.s_pos     = a.s_pos + b.s_pos;
    res.s_pos_off = a.s_pos_off + b.s_pos_off;
    res.s_tex_off = a.s_tex_off + b.s_tex_off;
    res.quality   = a.quality + b.quality;
    
    return res;
  }

  nniSample qwa(const std::vector<nniSample>&smpls){
    nniSample res;

    res.s_pos_cs[0] = 0.0;
    res.s_pos_cs[0] = 0.0;
    res.s_pos_cs[0] = 0.0;

    res.s_pos.x = 0.0;
    res.s_pos.y = 0.0;
    res.s_pos.z = 0.0;

    res.s_pos_off.x = 0.0;
    res.s_pos_off.y = 0.0;
    res.s_pos_off.z = 0.0;

    res.s_tex_off.u = 0.0;
    res.s_tex_off.v = 0.0;

    res.quality = 0.0;

    double weight = 0.0;
    for(const auto& s : smpls){
      weight += s.quality;
      res = res + (s.quality * s);
    }
    res = (1.0f / weight) * res;
   
    return res;
  }

}

Resampler::Resampler()
{}

Resampler::~Resampler()
{}



void
Resampler::resampleGridBased(std::vector<nniSample>& sps, const CalibVolume* cv, const char* basefilename){

  std::map<size_t, std::vector<nniSample>> grid;
  const unsigned scale = 1;
  for(const auto& s : sps){
    size_t grid_loc = ((std::round(s.s_pos.z/scale) * (cv->width/scale) * (cv->height/scale))
		       + (std::round(s.s_pos.y/scale) * (cv->width/scale)) + std::round(s.s_pos.x/scale));
    grid[grid_loc].push_back(s);
  }
  sps.clear();

  std::cerr << "grid size: " << grid.size() << " of " << cv->width * cv->height * cv->depth << std::endl;
  for(const auto& smpls : grid){
    nniSample tmp = qwa(smpls.second);
    if(! std::isnan(tmp.quality)){
      sps.push_back(tmp);
    }
    else{
      std::cerr << "INFO QWA sample is not valid!...skipping" << std::endl;
      for(const auto& s : smpls.second){
	std::cerr << "QWA sample was " << s << std::endl;
      }
    }
  }
  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! after quality weighted filterning applying "
	    << sps.size() << " for volume" << std::endl;


  if(0 != basefilename){
    // write samples coordinates to xyz file
    std::ofstream xyz_file_pos_cs(std::string(basefilename) + "_resampleGridBased_pos_cs.xyz");
    std::ofstream xyz_file_pos_vs(std::string(basefilename) + "_resampleGridBased_pos_vs.xyz");
    for(const auto& s : sps){
      xyz_file_pos_cs << s.s_pos_cs.x << " " << s.s_pos_cs.y << " " << s.s_pos_cs.z << " 255 255 255" << std::endl; 
      xyz_file_pos_vs << s.s_pos.x << " " << s.s_pos.y << " " << s.s_pos.z << " 255 255 255" << std::endl; 

    }
  }
}
  
void
Resampler::fillBorder(std::vector<nniSample>& sps, const CalibVolume* cv, const NearestNeighbourSearch* nns, const unsigned idwneighbours, const char* basefilename){


  const unsigned cv_width  = cv->width;
  const unsigned cv_height = cv->height;
  const unsigned cv_depth  = cv->depth;

  

  // 1. calculate bounding box around sps
  glm::vec3 sps_min = glm::vec3(std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max(),
				std::numeric_limits<float>::max());
  glm::vec3 sps_max = glm::vec3(std::numeric_limits<float>::lowest(),
				std::numeric_limits<float>::lowest(),
				std::numeric_limits<float>::lowest());

  for(const auto s : sps){
    sps_min[0] = std::min(sps_min[0], s.s_pos.x);
    sps_min[1] = std::min(sps_min[1], s.s_pos.y);
    sps_min[2] = std::min(sps_min[2], s.s_pos.z);

    sps_max[0] = std::max(sps_max[0], s.s_pos.x);
    sps_max[1] = std::max(sps_max[1], s.s_pos.y);
    sps_max[2] = std::max(sps_max[2], s.s_pos.z);
  }
  

  std::cerr << "sps bounding box: " << sps_min << " -> " << sps_max << std::endl;
  {

#if 0    
    const unsigned border   = 5;
    const unsigned cv_min_x = std::max(0, int(std::floor(sps_min[0]) - border));
    const unsigned cv_min_y = std::max(0, int(std::floor(sps_min[1]) - border));
    const unsigned cv_min_z = std::max(0, int(std::floor(sps_min[2]) - border));
    
    const unsigned cv_max_x = std::min(unsigned(std::ceil(sps_max[0]) + border), cv_width);
    const unsigned cv_max_y = std::min(unsigned(std::ceil(sps_max[1]) + border), cv_height);
    const unsigned cv_max_z = std::min(unsigned(std::ceil(sps_max[2]) + border), cv_depth);
#else
    const unsigned cv_min_x = 5;
    const unsigned cv_min_y = 5;
    const unsigned cv_min_z = 5;
    
    const unsigned cv_max_x = cv_width  - 5;
    const unsigned cv_max_y = cv_height - 5;
    const unsigned cv_max_z = cv_depth  - 5;
#endif
    

    std::cerr << "sps bounding box u: " << glm::vec3(cv_min_x, cv_min_y, cv_min_z) << " -> " << glm::vec3(cv_max_x, cv_max_y, cv_max_z) << std::endl;
    
    
    Coin mycoin(0.02f);
    RandRange myrange(-1.0,1.0);
    
    const glm::vec3 diameter(cv_max_x - cv_min_x, cv_max_y - cv_min_y, cv_max_z - cv_min_z);
    const float max_influence_dist = glm::length(diameter);
    
    
    for(unsigned z = cv_min_z; z < cv_max_z; ++z){
      for(unsigned y = cv_min_y; y < cv_max_y; ++y){
	for(unsigned x = cv_min_x; x < cv_max_x; ++x){
	  
	  // skip everything but border!
	  if( !(
		(z == cv_min_z || z == (cv_max_z - 1))  ||
		(y == cv_min_y || y == (cv_max_y - 1)) ||
		(x == cv_min_x || x == (cv_max_x - 1))
		)
	      ){
	    continue;
	  }
	  
	  if(!mycoin()){
	    continue;
	  }

	

	  nniSample support_sample;
	  support_sample.s_pos.x = x + myrange();
	  support_sample.s_pos.y = y + myrange();
	  support_sample.s_pos.z = z + myrange();
	  
	  support_sample.s_pos_off.x = 0.0;
	  support_sample.s_pos_off.y = 0.0;
	  support_sample.s_pos_off.z = 0.0;
	  
	  support_sample.s_tex_off.u = 0.0;
	  support_sample.s_tex_off.v = 0.0;
	  
	  std::vector<nniSample> neighbours = nns->search(support_sample,idwneighbours);
	  if(neighbours.empty()){
	    std::cerr << "ERROR in Resampler::fillBorder -> no neighbours found, skipping voxel at pos " << support_sample.s_pos << std::endl;
	    continue;
	  }
	  Calibrator::idw_interpolate(neighbours, idwneighbours, support_sample, max_influence_dist);
	  
	  xyz s_pos_cs = getTrilinear(cv->cv_xyz, cv_width, cv_height, cv_depth,
				      support_sample.s_pos.x, support_sample.s_pos.y, support_sample.s_pos.z) + support_sample.s_pos_off;
	  support_sample.s_pos_cs = glm::vec3(s_pos_cs.x, s_pos_cs.y, s_pos_cs.z);
	  support_sample.quality = 1.0; // ????
	  
	  sps.push_back(support_sample);
	  
	}
      }
    }
  }


  if(0 != basefilename){
    // write samples coordinates to xyz file
    std::ofstream xyz_file_pos_cs(std::string(basefilename) + "_fillBorder_pos_cs.xyz");
    std::ofstream xyz_file_pos_vs(std::string(basefilename) + "_fillBorder_pos_vs.xyz");
    for(const auto& s : sps){
      xyz_file_pos_cs << s.s_pos_cs.x << " " << s.s_pos_cs.y << " " << s.s_pos_cs.z << " 255 255 255" << std::endl; 
      xyz_file_pos_vs << s.s_pos.x << " " << s.s_pos.y << " " << s.s_pos.z << " 255 255 255" << std::endl; 

    }
  }
}
