#include "calibrator.hpp"


#include <NaturalNeighbourInterpolator.hpp>
#include <ChessboardSampling.hpp>
#include <PlaneFit.hpp>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>


#include <chrono>
#include <cmath>

#include <map>
#include <algorithm>
#include <unistd.h>
#include <limits>
#include <fstream>

#include <string.h> // memset

namespace{

  template <class T>
  inline std::string
  toString(T value)
  {
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }


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

/*static*/ bool Calibrator::using_nni = false;

Calibrator::Calibrator()
  : m_nni_possible(0)
{}


Calibrator::~Calibrator(){
  if(m_nni_possible){
    delete [] m_nni_possible;
  }
}



void
Calibrator::applySamples(CalibVolume* cv, const std::vector<samplePoint>& sps, const RGBDConfig& cfg, unsigned idwneighbours, const char* basefilename){

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



  std::vector<nniSample> nnisamples_nni;
  std::cerr << "grid size: " << grid.size() << " of " << cv->width * cv->height * cv->depth << std::endl;
  for(const auto& smpls : grid){
    nniSample tmp = qwa(smpls.second);
    if(! std::isnan(tmp.quality)){
      nnisamples.push_back(tmp);
      nnisamples_nni.push_back(tmp);
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


  std::cerr << "initializing nearest neighbor search for " << nnisamples.size() << " samples." << std::endl;
  NearestNeighbourSearch nns(nnisamples); // Be careful, nnisamples are just shared here!!!!

  // init calib volume for natural neighbor interpolation
  NaturalNeighbourInterpolator* nnip = 0;
  CalibVolume* cv_nni = 0;
  if(using_nni){
    if(m_nni_possible == 0){
      m_nni_possible = new unsigned char [cv->width * cv->height * cv->depth];
    }
    memset(m_nni_possible, 0, cv->width * cv->height * cv->depth);
    cv_nni = new CalibVolume(cv->width, cv->height, cv->depth, cv->min_d, cv->max_d);
    std::cerr << "initializing natural neighbor interpolation for " << nnisamples_nni.size() << " samples." << std::endl;
    std::shuffle(std::begin(nnisamples_nni), std::end(nnisamples_nni), std::default_random_engine());
    nnip =   new NaturalNeighbourInterpolator(nnisamples_nni);
  }

  const unsigned numthreads = 32;
  std::cerr << "start interpolation per thread for " << numthreads << " threads." << std::endl;

  boost::thread_group threadGroup;
    
  for (unsigned tid = 0; tid < numthreads; ++tid){
    threadGroup.create_thread(boost::bind(&Calibrator::applySamplesPerThread, this, cv, &nns, tid, numthreads, idwneighbours, cv_nni, nnip));
  }
  threadGroup.join_all();

  
  if(using_nni){
    blendIDW2NNI(cv, cv_nni, basefilename);
  }

  auto end_time = std::chrono::system_clock::now();
  std::cerr << "finished interpolation in "
	    <<  std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count()
	    << " seconds" << std::endl;

}


void
Calibrator::blendIDW2NNI(CalibVolume* cv, CalibVolume* cv_nni, const char* basefilename){


  // print stats for nni_possible and write to volume file
  size_t possible = 0;
  for(size_t idx = 0; idx < (cv->width * cv->height * cv->depth); ++idx){
    possible += m_nni_possible[idx] > 0 ? 1 : 0;
  }
  std::cout << "natural neighbor interpolation was possible for: "
	    << possible << " out of: " << (cv->width * cv->height * cv->depth)
	    << std::endl;

  // 1st pass detect border
  unsigned char* nni_border = new unsigned char[cv->width * cv->height * cv->depth];
  memset(nni_border, 0, cv->width * cv->height * cv->depth);
  const int border_size = 1;
  for(unsigned z = 0; z < cv->depth; ++z){
    for(unsigned y = 0; y < cv->height; ++y){
      for(unsigned x = 0; x < cv->width; ++x){
	const unsigned cv_index = (z * cv->width * cv->height) + (y * cv->width) + x;
	if(m_nni_possible[cv_index]){
	  bool border = false;
	  for(int z_l = std::max(0 , (int(z) - border_size));
	      z_l < std::min(int(cv->depth), (int(z) + border_size + 1));
	      ++z_l){
	    for(int y_l = std::max(0 , (int(y) - border_size));
		y_l < std::min(int(cv->height), (int(y) + border_size + 1));
		++y_l){
	      for(int x_l = std::max(0 , (int(x) - border_size));
		  x_l < std::min(int(cv->width), (int(x) + border_size + 1));
		  ++x_l){
		const unsigned cv_index_l = (z_l * cv->width * cv->height) + (y_l * cv->width) + x_l;
		if(0 == m_nni_possible[cv_index_l]){
		  border = true;
		}
	      }
	    }
	  }
	  nni_border[cv_index] = border ? 255 : 0;
	}
      }
    }
  }

  // 2nd pass blend between NNI and IDW
  unsigned char* nni_percentage = new unsigned char [cv->width * cv->height * cv->depth];
  memset(nni_percentage, 0, cv->width * cv->height * cv->depth);
  const int kernel_size = 8;
  const float min_dist = 1.0f;
  const float max_dist = 6.0f;

  for(unsigned z = 0; z < cv->depth; ++z){
    for(unsigned y = 0; y < cv->height; ++y){
      for(unsigned x = 0; x < cv->width; ++x){
	const unsigned cv_index = (z * cv->width * cv->height) + (y * cv->width) + x;
	if(m_nni_possible[cv_index]){
	  float distance_to_border = std::numeric_limits<float>::max();
	  glm::vec3 voxel_curr(1.0f * x, 1.0f * y, 1.0f * z);
	  for(int z_l = std::max(0 , (int(z) - kernel_size));
	      z_l < std::min(int(cv->depth), (int(z) + kernel_size + 1));
	      ++z_l){
	    for(int y_l = std::max(0 , (int(y) - kernel_size));
		y_l < std::min(int(cv->height), (int(y) + kernel_size + 1));
		++y_l){
	      for(int x_l = std::max(0 , (int(x) - kernel_size));
		  x_l < std::min(int(cv->width), (int(x) + kernel_size + 1));
		  ++x_l){
		const unsigned cv_index_l = (z_l * cv->width * cv->height) + (y_l * cv->width) + x_l;
		if(nni_border[cv_index_l] > 0){
		  distance_to_border = std::min(distance_to_border,
						glm::length(voxel_curr - glm::vec3(1.0f * x_l, 1.0f * y_l, 1.0f * z_l)));
		}
	      }
	    }
	  }
  
	  const float t_blend = std::min( max_dist, std::max(min_dist, distance_to_border)) / max_dist;
	  if(t_blend > 1.0f || t_blend < 0.0f){
	    std::cerr << "ERROR: invalid t_blend: " << t_blend << std::endl;
	    exit(0);
	  }

	  nni_percentage[cv_index] = (unsigned char) std::max(0.0f, std::min(255.0f, 255.0f * t_blend));
	  const xyz xyz_idw = cv->cv_xyz[cv_index];
	  const xyz xyz_nni = cv_nni->cv_xyz[cv_index];
	  
	  const uv uv_idw = cv->cv_uv[cv_index];
	  const uv uv_nni = cv_nni->cv_uv[cv_index];
	  
	  cv->cv_xyz[cv_index] = interpolate(xyz_idw, xyz_nni, t_blend);
	  cv->cv_uv[cv_index]  = interpolate(uv_idw , uv_nni , t_blend);

	}
      }
    }
  }

  FILE* f_nni_stats = fopen((std::string(basefilename) + "_nnistats").c_str(), "wb");
  fwrite(m_nni_possible, sizeof(unsigned char), (cv->width * cv->height * cv->depth), f_nni_stats);
  fclose(f_nni_stats);
  
  FILE* f_nni_border = fopen((std::string(basefilename) + "_nniborder").c_str(), "wb");
  fwrite(nni_border, sizeof(unsigned char), (cv->width * cv->height * cv->depth), f_nni_border);
  fclose(f_nni_border);
  
  FILE* f_nni_percentage = fopen((std::string(basefilename) + "_percentage").c_str(), "wb");
  fwrite(nni_percentage, sizeof(unsigned char), (cv->width * cv->height * cv->depth), f_nni_percentage);
  fclose(f_nni_percentage);

  delete [] nni_border;
  delete [] nni_percentage;

}

void
Calibrator::evaluateSamples(CalibVolume* cv, std::vector<samplePoint>& sps, const RGBDConfig& cfg, const char* basefilename, bool isnni){

  unsigned char* cv_nnistats = 0;
  unsigned nni_valids = 0;
  if(isnni){
    cv_nnistats = new unsigned char [cv->width * cv->height * cv->depth];
    // load .cv_nnistats if nni has to be evaluated
    FILE* f_nni_stats = fopen((std::string(basefilename) + "_nnistats").c_str(), "rb");
    fread(cv_nnistats, sizeof(unsigned char), (cv->width * cv->height * cv->depth), f_nni_stats);
    fclose(f_nni_stats);
  }

  std::vector<nniSample> nnisamples_error_vol;

  std::vector<float> errors_3D;
  std::vector<float> errors_2D;
  float max_3D = std::numeric_limits<float>::lowest();
  float max_2D = std::numeric_limits<float>::lowest();
  
  for(unsigned i = 0; i < sps.size(); ++i){
    const unsigned cv_width = cv->width;
    const unsigned cv_height = cv->height;
    const unsigned cv_depth = cv->depth;

    const float x = cv_width  *  ( sps[i].tex_depth.u) / cfg.size_d.x;
    const float y = cv_height *  ( sps[i].tex_depth.v)/ cfg.size_d.y;
    const float z = cv_depth  *  ( sps[i].depth - cv->min_d)/(cv->max_d - cv->min_d);


    if(isnni){
      const unsigned cv_index000 = (std::floor(z) * cv_width * cv_height) + (std::floor(y) * cv_width) + std::floor(x);
      const unsigned cv_index001 = (std::floor(z) * cv_width * cv_height) + (std::floor(y) * cv_width) + std::ceil(x);
      const unsigned cv_index010 = (std::floor(z) * cv_width * cv_height) + (std::ceil(y) * cv_width) + std::floor(x);
      const unsigned cv_index011 = (std::floor(z) * cv_width * cv_height) + (std::ceil(y) * cv_width) + std::ceil(x);
      const unsigned cv_index100 = (std::ceil(z) * cv_width * cv_height) + (std::floor(y) * cv_width) + std::floor(x);
      const unsigned cv_index101 = (std::ceil(z) * cv_width * cv_height) + (std::floor(y) * cv_width) + std::ceil(x);
      const unsigned cv_index110 = (std::ceil(z) * cv_width * cv_height) + (std::ceil(y) * cv_width) + std::floor(x);
      const unsigned cv_index111 = (std::ceil(z) * cv_width * cv_height) + (std::ceil(y) * cv_width) + std::ceil(x);
      bool nni_valid  = ( cv_nnistats[cv_index000]
			  && cv_nnistats[cv_index001]
			  && cv_nnistats[cv_index010]
			  && cv_nnistats[cv_index011]
			  && cv_nnistats[cv_index100]
			  && cv_nnistats[cv_index101]
			  && cv_nnistats[cv_index110]
			  && cv_nnistats[cv_index111]);
      if(!nni_valid){
	continue;
      }
      else{
	++nni_valids;
      }
    }

    xyz pos = getTrilinear(cv->cv_xyz, cv_width, cv_height, cv_depth, x , y , z );
    uv  tex = getTrilinear(cv->cv_uv,  cv_width, cv_height, cv_depth, x , y , z );

    sps[i].pos_offset.x = sps[i].pos_real[0] - pos.x;
    sps[i].pos_offset.y = sps[i].pos_real[1] - pos.y;
    sps[i].pos_offset.z = sps[i].pos_real[2] - pos.z;
      
    sps[i].tex_offset.u = sps[i].tex_color.u/cfg.size_rgb.x - tex.u;
    sps[i].tex_offset.v = sps[i].tex_color.v/cfg.size_rgb.y - tex.v;

    sps[i].quality = 1.0f;

    const float err_3D(glm::length(glm::vec3(sps[i].pos_offset.x,sps[i].pos_offset.y,sps[i].pos_offset.z)));
    const float err_2D(glm::length(glm::vec3(sps[i].tex_offset.u * cfg.size_rgb.x,
					      sps[i].tex_offset.v * cfg.size_rgb.y,0.0)));

    errors_3D.push_back(err_3D);
    errors_2D.push_back(err_2D);
   
    max_3D = std::max(max_3D, err_3D);
    max_2D = std::max(max_2D, err_2D);

    // track local error for volume vis
    nniSample nnis;
    nnis.s_tex_off.u = err_3D;
    nnis.s_tex_off.v = err_2D;
    
    nnis.s_pos.x = x;
    nnis.s_pos.y = y;
    nnis.s_pos.z = z;
    
    nnisamples_error_vol.push_back(nnis);

  }

  double mean3D, mean2D, sd3D, sd2D;
  calcMeanSD(errors_3D, mean3D, sd3D);
  calcMeanSD(errors_2D, mean2D, sd2D);
  std::cout << "mean_error_3D: " << mean3D << " [" << sd3D << "] (" << max_3D << ") (in meter)" << std::endl;
  std::cout << "mean_error_2D: " << mean2D << " [" << sd2D << "] (" << max_2D << ") (in pixels)" << std::endl;
  if(isnni){
    std::cout << "could evaluate " << nni_valids << " samples from " << sps.size() << std::endl;
  }
  createErrorVis(nnisamples_error_vol, cv->width, cv->height, cv->depth, basefilename);

}

void
Calibrator::createErrorVis(const std::vector<nniSample>& sps, const unsigned width, const unsigned height, const unsigned depth, const char* basefilename){

  // create volumes for world and texture coordinates
  unsigned char* error_vol_3D = new unsigned char [width * height * depth];
  memset(error_vol_3D, 0, width * height * depth);
  
  unsigned char* error_vol_2D = new unsigned char [width * height * depth];
  memset(error_vol_2D, 0, width * height * depth);

  unsigned char* error_vol_3D_nni = new unsigned char [width * height * depth];
  memset(error_vol_3D_nni, 0, width * height * depth);
  
  unsigned char* error_vol_2D_nni = new unsigned char [width * height * depth];
  memset(error_vol_2D_nni, 0, width * height * depth);

  
  NearestNeighbourSearch nns(sps);
  std::vector<nniSample> sps_nni(sps);
  std::shuffle(std::begin(sps_nni), std::end(sps_nni), std::default_random_engine());
  NaturalNeighbourInterpolator nnip(sps_nni);


  // boost threads here
  const unsigned numthreads = 32;
  std::cerr << "start interpolation of error visualization volumes using " << numthreads << " threads." << std::endl;
  boost::thread_group threadGroup;
  for (unsigned tid = 0; tid < numthreads; ++tid){
    //threadGroup.create_thread(boost::bind(&Calibrator::applyErrorVisPerThread, this, width, height, depth,
    //					  error_vol_3D, error_vol_2D, error_vol_3D_nni, error_vol_2D_nni, &nns, &nnip, tid, numthreads));
    threadGroup.create_thread([width, height, depth,
			       error_vol_3D, error_vol_2D, error_vol_3D_nni, error_vol_2D_nni, &nns, &nnip, tid, numthreads, this](){
				this->applyErrorVisPerThread(width, height, depth,
							     error_vol_3D, error_vol_2D, error_vol_3D_nni, error_vol_2D_nni, &nns, &nnip, tid, numthreads);
			      });
  }
  threadGroup.join_all();

  // write error volumes to /tmp

  FILE* f_error_vol_3D = fopen((std::string(basefilename) + "_error3D_idw").c_str(), "wb");
  fwrite(error_vol_3D, sizeof(unsigned char), (width * height * depth), f_error_vol_3D);
  fclose(f_error_vol_3D);

  FILE* f_error_vol_2D = fopen((std::string(basefilename) + "_error2D_idw").c_str(), "wb");
  fwrite(error_vol_2D, sizeof(unsigned char), (width * height * depth), f_error_vol_2D);
  fclose(f_error_vol_2D);


  FILE* f_error_vol_3Dnni = fopen((std::string(basefilename) + "_error3D").c_str(), "wb");
  fwrite(error_vol_3D_nni, sizeof(unsigned char), (width * height * depth), f_error_vol_3Dnni);
  fclose(f_error_vol_3Dnni);

  FILE* f_error_vol_2Dnni = fopen((std::string(basefilename) + "_error2D").c_str(), "wb");
  fwrite(error_vol_2D_nni, sizeof(unsigned char), (width * height * depth), f_error_vol_2Dnni);
  fclose(f_error_vol_2Dnni);

}


void
Calibrator::applyErrorVisPerThread(const unsigned width, const unsigned height, const unsigned depth,
				   unsigned char* error_vol_3D, unsigned char* error_vol_2D,
				   unsigned char* error_vol_3D_nni, unsigned char* error_vol_2D_nni,
				   const NearestNeighbourSearch* nns, const NaturalNeighbourInterpolator* nnip,
				   const unsigned tid, const unsigned numthreads){

  const unsigned idwneighbours = 20;
  const glm::vec3 diameter(width, height, depth);
  const float max_influence_dist = glm::length(diameter);
  const float max_error_3D_vol = 0.01; // 5cm
  const float max_error_2D_vol = 2.0; // 5 pixel

  for(unsigned z = tid; z < depth; z += numthreads){
    for(unsigned y = 0; y < height; ++y){
      for(unsigned x = 0; x < width; ++x){
	
	const unsigned cv_index = (z * width * height) + (y * width) + x;
	
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
	idw_interpolate(neighbours, idwneighbours, ipolant, max_influence_dist);

	// s_tex_off.u -> 3D error
	error_vol_3D[cv_index] = (unsigned char) (std::max(0.0f,
							   std::min(ipolant.s_tex_off.u, max_error_3D_vol))
						  * 255.0f/max_error_3D_vol);
	// s_tex_off.v -> 2D error
	error_vol_2D[cv_index] = (unsigned char) (std::max(0.0f,
							   std::min(ipolant.s_tex_off.v, max_error_2D_vol))
						  * 255.0f/max_error_2D_vol);


	{
	  nniSample ipolant_nni;
	  ipolant_nni.s_pos.x = x;
	  ipolant_nni.s_pos.y = y;
	  ipolant_nni.s_pos.z = z;
	  
	  ipolant_nni.s_pos_off.x = 0.0;
	  ipolant_nni.s_pos_off.y = 0.0;
	  ipolant_nni.s_pos_off.z = 0.0;
	  
	  ipolant_nni.s_tex_off.u = 0.0;
	  ipolant_nni.s_tex_off.v = 0.0;

	  bool nni_valid = nnip->interpolate(ipolant_nni);
	  if(nni_valid){
	    // s_tex_off.u -> 3D error
	    error_vol_3D_nni[cv_index] = (unsigned char) (std::max(0.0f,
								   std::min(ipolant_nni.s_tex_off.u, max_error_3D_vol))
							  * 255.0f/max_error_3D_vol);
	    // s_tex_off.v -> 2D error
	    error_vol_2D_nni[cv_index] = (unsigned char) (std::max(0.0f,
								   std::min(ipolant_nni.s_tex_off.v, max_error_2D_vol))
							  * 255.0f/max_error_2D_vol);
	  }

	}


      }
    }
  }
  
}





double
Calibrator::evaluatePlanes(CalibVolume* cv, ChessboardSampling* cbs, const RGBDConfig& cfg, unsigned stride){

  const unsigned cv_width = cv->width;
  const unsigned cv_height = cv->height;
  const unsigned cv_depth = cv->depth;


  std::vector<double> plane_qualities;
  const std::vector<ChessboardRange>& valid_ranges = cbs->getValidRanges();
  const std::vector<ChessboardViewIR>& cb_irs = cbs->getIRs();
  for(const auto& r : valid_ranges){
    for(unsigned cb_id = r.start; cb_id != r.end; ++cb_id){
      if((cb_id % stride) == 0){
	std::vector<xyz> world_space_corners;
	for(unsigned idx = 0; idx < (CB_WIDTH * CB_HEIGHT); ++idx){
	  
	  const xyz corner = cb_irs[cb_id].corners[idx];
	  const float x = cv_width  *  ( corner.x)/ cfg.size_d.x;
	  const float y = cv_height *  ( corner.y)/ cfg.size_d.y;
	  const float z = cv_depth  *  ( corner.z - cv->min_d)/(cv->max_d - cv->min_d);
	  xyz pos = getTrilinear(cv->cv_xyz, cv_width, cv_height, cv_depth, x , y , z );
	  
	  pos.x *= 10;
	  pos.y *= 10;
	  pos.z *= 10;
	  
	  world_space_corners.push_back(pos);
	}
	
	const auto pq = detectPlaneQuality(world_space_corners);
	//std::cout << "cb_id: " << cb_id << " -> " << pq << std::endl;
	plane_qualities.push_back(pq);
      }

    }
  }
  
  double mean;
  double sd;
  calcMeanSD(plane_qualities, mean, sd);

  return mean;
}

double
Calibrator::evaluateShapes(CalibVolume* cv, ChessboardSampling* cbs, const RGBDConfig& cfg, unsigned stride){

  const unsigned cv_width = cv->width;
  const unsigned cv_height = cv->height;
  const unsigned cv_depth = cv->depth;

  const float gt_area = 6.0 * 4.0 * 0.075 * 0.075;
  std::vector<double> shape_qualities;
  const std::vector<ChessboardRange>& valid_ranges = cbs->getValidRanges();
  const std::vector<ChessboardViewIR>& cb_irs = cbs->getIRs();
  for(const auto& r : valid_ranges){
    for(unsigned cb_id = r.start; cb_id != r.end; ++cb_id){
      if((cb_id % stride) == 0){

	ChessboardViewIR tmp_ir;
	
	for(unsigned idx = 0; idx < (CB_WIDTH * CB_HEIGHT); ++idx){
	  
	  const xyz corner = cb_irs[cb_id].corners[idx];
	  const float x = cv_width  *  ( corner.x)/ cfg.size_d.x;
	  const float y = cv_height *  ( corner.y)/ cfg.size_d.y;
	  const float z = cv_depth  *  ( corner.z - cv->min_d)/(cv->max_d - cv->min_d);

	  tmp_ir.corners[idx] = getTrilinear(cv->cv_xyz, cv_width, cv_height, cv_depth, x , y , z );
	  
	}
	
	const auto st = tmp_ir.calcShapeStats3D();
	double area = 0.0;
	for(const auto& a : st.areas){
	  area += a;
	}
	const double sq = 1.0 - (std::abs(gt_area - area)/gt_area);
	shape_qualities.push_back(sq);
      }

    }
  }
  
  double mean;
  double sd;
  calcMeanSD(shape_qualities, mean, sd);

  return mean;
}



void
Calibrator::applySamplesPerThread(CalibVolume* cv, const NearestNeighbourSearch* nns, unsigned tid, unsigned numthreads, unsigned idwneighbours, CalibVolume* cv_nni, const NaturalNeighbourInterpolator* nnip){


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
	  idw_interpolate(neighbours, idwneighbours, ipolant, max_influence_dist);

	  const xyz xyz_curr = cv->cv_xyz[cv_index];
	  const uv  uv_curr  = cv->cv_uv[cv_index];

	  cv->cv_xyz[cv_index] = cv->cv_xyz[cv_index] + ipolant.s_pos_off;
	  cv->cv_uv[cv_index]  = cv->cv_uv[cv_index]  + ipolant.s_tex_off;
	  
	  if(using_nni){
	    nniSample ipolant_nni;
	    ipolant_nni.s_pos.x = x;
	    ipolant_nni.s_pos.y = y;
	    ipolant_nni.s_pos.z = z;
	    
	    ipolant_nni.s_pos_off.x = 0.0;
	    ipolant_nni.s_pos_off.y = 0.0;
	    ipolant_nni.s_pos_off.z = 0.0;
	    
	    ipolant_nni.s_tex_off.u = 0.0;
	    ipolant_nni.s_tex_off.v = 0.0;
	    bool nni_valid = nnip->interpolate(ipolant_nni);
	    m_nni_possible[cv_index] = nni_valid ? 255 : 0;
	    if(nni_valid){
	      cv_nni->cv_xyz[cv_index] = xyz_curr + ipolant_nni.s_pos_off;
	      cv_nni->cv_uv[cv_index]  = uv_curr  + ipolant_nni.s_tex_off;
	      // STEPPO REMOVE
	      //std::cerr << "IDW: " << ipolant << std::endl;
	      //std::cerr << "NNI: " << ipolant_nni << std::endl;
	    }
	  }

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
