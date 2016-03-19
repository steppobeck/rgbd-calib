#include "resampler.hpp"

#include <calibvolume.hpp>
#include <NearestNeighbourSearch.hpp>
#include <calibrator.hpp>
#include <map>
#include <fstream>


namespace{

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
  for(const auto& s : sps){
    size_t grid_loc = ((std::round(s.s_pos.z) * cv->width * cv->height)
		       + (std::round(s.s_pos.y) * cv->width) + std::round(s.s_pos.x));
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


  const glm::vec3 diameter(cv->width, cv->height, cv->depth);
  const float max_influence_dist = glm::length(diameter);

  const unsigned cv_width  = cv->width;
  const unsigned cv_height = cv->height;
  const unsigned cv_depth  = cv->depth;
  for(unsigned z = 0; z < cv_depth; ++z){
    for(unsigned y = 0; y < cv_height; ++y){
      for(unsigned x = 0; x < cv_width; ++x){
	
	// skip everything but border!
	if( !(
	      (z == 0 || z == (cv_depth - 1))  ||
	      (y == 0 || y == (cv_height - 1)) ||
	      (x == 0 || x == (cv_width - 1))
	       )
	    ){
	  continue;
	}

	const unsigned cv_index = (z * cv_width * cv_height) + (y * cv_width) + x;

	nniSample ipolant;
	ipolant.s_pos.x = x;
	ipolant.s_pos.y = y;
	ipolant.s_pos.z = z;
	
	ipolant.s_pos_off.x = 0.0;
	ipolant.s_pos_off.y = 0.0;
	ipolant.s_pos_off.z = 0.0;
	
	ipolant.s_tex_off.u = 0.0;
	ipolant.s_tex_off.v = 0.0;
	  
	std::vector<nniSample> neighbours = nns->search(ipolant,idwneighbours);
	if(neighbours.empty()){
	  std::cerr << "ERROR in Resampler::fillBorder -> no neighbours found, skipping voxel at pos " << ipolant.s_pos << std::endl;
	  continue;
	}
	Calibrator::idw_interpolate(neighbours, idwneighbours, ipolant, max_influence_dist);

	xyz s_pos_cs = cv->cv_xyz[cv_index] + ipolant.s_pos_off;
	ipolant.s_pos_cs = glm::vec3(s_pos_cs.x, s_pos_cs.y, s_pos_cs.z);
	ipolant.quality = 1.0; // ????

	sps.push_back(ipolant);

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
