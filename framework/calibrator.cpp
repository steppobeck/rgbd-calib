#include "calibrator.hpp"


#include <glm/gtc/type_ptr.hpp>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


#include <chrono>
#include <cmath>

#include <map>
#include <algorithm>
#include <unistd.h>

#include <fstream>


namespace{


  float
  gauss(float x, float sigma, float mean){
    return (1.0f/(sigma*sqrt(2.0f * M_PI))) * exp( -0.5f * ((x-mean)/sigma) * ((x-mean)/sigma) );
  }

  nniSample operator * (float q, const nniSample& s){
    nniSample res;
    res.s_pos     = q * s.s_pos;
    res.s_pos_off = q * s.s_pos_off;
    res.s_tex_off = q * s.s_tex_off;
    res.quality   = q * s.quality;
    
    return res;
  }

  nniSample operator + (const nniSample& a, const nniSample& b){
    nniSample res;
    res.s_pos     = a.s_pos + b.s_pos;
    res.s_pos_off = a.s_pos_off + b.s_pos_off;
    res.s_tex_off = a.s_tex_off + b.s_tex_off;
    res.quality   = a.quality + b.quality;
    
    return res;
  }

  nniSample qwa(const std::vector<nniSample>&smpls){
    nniSample res;
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


Calibrator::Calibrator()
{}


Calibrator::~Calibrator()
{}



void
Calibrator::applySamples(CalibVolume* cv, const char* filename, const RGBDConfig& cfg, unsigned idwneighbours){


  // load samples from filename
  std::vector<samplePoint> sps;

  std::ifstream iff(filename, std::ifstream::binary);
  const unsigned num_samples_in_file = calcNumFrames(iff,
						     sizeof(float) +
						     sizeof(uv) +
						     sizeof(uv) +
						     sizeof(xyz) +
						     sizeof(uv) +
						     sizeof(glm::vec3) +
						     sizeof(float));
  for(unsigned i = 0; i < num_samples_in_file; ++i){
    samplePoint s;
    iff.read((char*) &s.depth, sizeof(float));
    iff.read((char*) &s.tex_color, sizeof(uv));
    iff.read((char*) &s.tex_depth, sizeof(uv));
    iff.read((char*) &s.pos_offset, sizeof(xyz));
    iff.read((char*) &s.tex_offset, sizeof(uv));
    iff.read((char*) glm::value_ptr(s.pos_real), sizeof(glm::vec3));
    iff.read((char*) &s.quality, sizeof(float));
    sps.push_back(s);
  }
  iff.close();  



  auto start_time = std::chrono::system_clock::now();
  //CGAL : build Tree, search 100 neighbors, try NNI of neighbourhood, fallback to IDW small neighborhood
  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! applying " << sps.size() << " for volume" << std::endl;

  std::vector<nniSample> nnisamples;

  for(unsigned s = 0; s < sps.size(); ++s){
    
    nniSample nnis;
    nnis.quality = sps[s/*sample point*/].quality;
    nnis.s_pos_off = sps[s/*sample point*/].pos_offset;
    nnis.s_tex_off  = sps[s/*sample point*/].tex_offset;
    // calculate distance from volume pos to sample pos
    
    nnis.s_pos.x = cv->width *  ( sps[s].tex_depth.u)/ cfg.size_d.x;
    nnis.s_pos.y = cv->height *  ( sps[s].tex_depth.v)/ cfg.size_d.y;
    nnis.s_pos.z = cv->depth  *  (sps[s].depth - cv->min_d) / (cv->max_d - cv->min_d);
    
    //std::cerr << s << " " << nnis << std::endl;
    nnisamples.push_back(nnis);
  }


  // build virtual grid
  std::map<size_t, std::vector<nniSample>> grid;
  for(const auto& s : nnisamples){
    size_t grid_loc = ((std::round(s.s_pos.z) * cv->width * cv->height)
		       + (std::round(s.s_pos.y) * cv->width) + std::round(s.s_pos.x));
    grid[grid_loc].push_back(s);
  }
  nnisamples.clear();
  std::cerr << "grid size: " << grid.size() << " of " << cv->width * cv->height * cv->depth << std::endl;
  for(const auto& smpls : grid){
    nniSample tmp = qwa(smpls.second);
    if(! std::isnan(tmp.quality)){
      nnisamples.push_back(tmp);
    }
    else{
      std::cerr << "INFO QWA sample is not valid!...skipping" << std::endl;
      for(const auto& s : smpls.second){
	std::cerr << "QWA sample was " << s << std::endl;
      }
    }
  }
  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! after quality weighted filterning applying "
	    << nnisamples.size() << " for volume" << std::endl;


  std::cerr << "initializing nearest neighbor search from " << nnisamples.size() << " samples." << std::endl;
  NearestNeighbourSearch nns(nnisamples);
  const unsigned numthreads = 24;
  std::cerr << "start interpolation per thread for " << numthreads << std::endl;
  boost::thread_group threadGroup;
    
  for (unsigned tid = 0; tid < numthreads; ++tid){
    threadGroup.create_thread(boost::bind(&Calibrator::applySamplesPerThread, this, cv, &nns, tid, numthreads, idwneighbours));
  }
  threadGroup.join_all();
  auto end_time = std::chrono::system_clock::now();
  std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! done, applying took seconds: "
	    <<  std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count() << std::endl;

}



void
Calibrator::applySamplesPerThread(CalibVolume* cv, const NearestNeighbourSearch* nns, unsigned tid, unsigned numthreads, unsigned idwneighbours){


  const glm::vec3 diameter(cv->width, cv->height, cv->depth);
  const float max_influence_dist = glm::length(diameter);

  const unsigned cv_width  = cv->width;
  const unsigned cv_height = cv->height;
  const unsigned cv_depth  = cv->depth;
    //unsigned having = 0;  
    for(unsigned z = tid; z < cv_depth; z += numthreads){
      //std::cerr << "tid: having " << ++having << " from " << cv_depth / numthreads <<  std::endl;
      for(unsigned y = 0; y < cv_height; ++y){
	for(unsigned x = 0; x < cv_width; ++x){
	  
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
	    std::cerr << "ERROR in Calibrator::applySamplesPerThread -> no neighbours found, skipping voxel at pos " << ipolant.s_pos << std::endl;
	    continue;
	  }

	  // NNI possible?
	  //std::vector<nniSample> neighbours = nns->search(ipolant,50);
	  //std::shuffle(std::begin(neighbours), std::end(neighbours), std::default_random_engine());
	  //NaturalNeighbourInterpolator nni(neighbours);
	  //bool nni_valid = nni.interpolate(ipolant);

	  bool nni_valid = false;
	  if(!nni_valid){
	    idw_interpolate(neighbours, idwneighbours, ipolant, max_influence_dist);
	  }
	  cv->cv_xyz[cv_index] = cv->cv_xyz[cv_index] + ipolant.s_pos_off;
	  cv->cv_uv[cv_index]  = cv->cv_uv[cv_index]  + ipolant.s_tex_off;

	}
      }
    }


}


void
Calibrator::idw_interpolate(const std::vector<nniSample>& neighbours, unsigned idw_neigbours, nniSample& ipolant, const float max_influence_dist){
    

  const float sigma = max_influence_dist * 1.0/3.3;
  const float mean = 0;
  const float norm = 1.0f/gauss(0.0f, sigma, mean);


  double weight_d = 0.0;
  xyz_d pos_offset;
  pos_offset.x = 0.0;pos_offset.y = 0.0;pos_offset.z = 0.0;
  uv_d tex_offset;
  tex_offset.u = 0.0;tex_offset.v = 0.0;
  glm::vec3 ipolant_pos(ipolant.s_pos.x,ipolant.s_pos.y,ipolant.s_pos.z);

  for(unsigned i = 0; i < neighbours.size() && i < idw_neigbours; ++i){
    nniSample s = neighbours[i];
    //std::cerr << "s.quality: " << s.quality <<  std::endl;
    glm::vec3 s_pos(s.s_pos.x,s.s_pos.y,s.s_pos.z);
    const float influence_dist = std::min(max_influence_dist, glm::length(ipolant_pos - s_pos));
    //std::cerr << "influence dist: " << influence_dist << " of " << max_influence_dist <<  std::endl;
    const double s_weight = /*double(s.quality) * */gauss(influence_dist, sigma, mean) * norm;
    //std::cerr << "s_weight: " << s_weight << std::endl;
    
    weight_d += s_weight;
    pos_offset = pos_offset + s_weight * s.s_pos_off;
    tex_offset = tex_offset + s_weight * s.s_tex_off;
    
  }

  if(weight_d > 0.00001){
    
    ipolant.s_pos_off.x = pos_offset.x/weight_d;
    ipolant.s_pos_off.y = pos_offset.y/weight_d;
    ipolant.s_pos_off.z = pos_offset.z/weight_d;

    ipolant.s_tex_off.u = tex_offset.u/weight_d;
    ipolant.s_tex_off.v = tex_offset.v/weight_d;
  }
  else{
    std::cerr << "ERROR in Calibrator::idw_interpolate!!!! weight to low (weight_d < 0.0001) " << std::endl;
  }
  
}
