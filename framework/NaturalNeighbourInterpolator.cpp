#include "NaturalNeighbourInterpolator.hpp"



  extern std::ostream& operator<< (std::ostream& o, const nniSample& s){
    o << "s_pos_cs: (" << s.s_pos_cs.x << "," << s.s_pos_cs.y << "," << s.s_pos_cs.z << ") "
      << "s_pos: (" << s.s_pos.x << "," << s.s_pos.y << "," << s.s_pos.z << ") "
      << "s_pos_off: (" << s.s_pos_off.x << "," << s.s_pos_off.y << "," << s.s_pos_off.z << ") "
      << "s_tex_off: (" << s.s_tex_off.u << "," << s.s_tex_off.v << ")"
      << "s_quality: " << s.quality;
    return o;
  }
	
	
  NaturalNeighbourInterpolator::NaturalNeighbourInterpolator(const std::vector<nniSample>& samples)
    : m_dt(),
      m_vd()
  {

    for(unsigned i = 0; i < samples.size(); ++i){

      nniSample sp = samples[i];
      Point3 sp_dt(sp.s_pos.x, sp.s_pos.y, sp.s_pos.z);
      m_vd[m_dt.insert(sp_dt)] = sp;

    }

  }

  NaturalNeighbourInterpolator::~NaturalNeighbourInterpolator(){
  }
    
  bool
  NaturalNeighbourInterpolator::interpolate(nniSample& ipolant) const{


    Point3 ipos(ipolant.s_pos.x, ipolant.s_pos.y, ipolant.s_pos.z);

    std::vector< std::pair< Vertex_handle,NT> > coor_sibson;
    NT norm_coeff_sibson;



    sibson_natural_neighbor_coordinates_3(m_dt,ipos,
					  std::back_inserter(coor_sibson),
					  norm_coeff_sibson);

    

    if(0 == coor_sibson.size()){
      return false;
    }



    //std::cerr << "norm_coeff: " << norm_coeff_sibson << " num_neighbours "<< coor_sibson.size() << std::endl;


    std::vector< std::pair< Vertex_handle,NT> >::iterator it;
    unsigned c_idx = 0;
    //double sum = 0;

    xyz_d pos_off;
    pos_off.x = 0.0;pos_off.y = 0.0;pos_off.z = 0.0;
    uv_d tex_off;
    tex_off.u = 0.0;tex_off.v = 0.0;

    for(it = coor_sibson.begin() ; it != coor_sibson.end() ; ++it, ++c_idx){
      double contribution_i = it->second;

      auto iter = m_vd.find(it->first);
      if(iter == m_vd.end()){
	return false;
      }

      nniSample s = iter->second; //nniSample s = m_vd[it->first];

#if 0
      glm::vec3 pos_glm(s.s_pos_off.x,s.s_pos_off.y,s.s_pos_off.z);
      const float od = glm::length(pos_glm);
      if(od > 0.1){
		std::cerr << "ERROR: offset_too_large at " << s.s_pos_off << " -> " << od << std::endl;
      }
#endif

      pos_off.x += contribution_i * s.s_pos_off.x;
      pos_off.y += contribution_i * s.s_pos_off.y;
      pos_off.z += contribution_i * s.s_pos_off.z;

      tex_off.u += contribution_i * s.s_tex_off.u;
      tex_off.v += contribution_i * s.s_tex_off.v;

      //sum += contribution_i;
      //std::cout << c_idx << " " << contribution_i << "% from " << it->first->point() << std::endl;
      //sum_x += it->second*(it->first->point().x());
      //sum_y += it->second*(it->first->point().y());
      //sum_z += it->second*(it->first->point().z());
    }
    //std::cerr << "sum: " << sum << std::endl;
    
    // normalize
    ipolant.s_pos_off.x = pos_off.x / norm_coeff_sibson;
    ipolant.s_pos_off.y = pos_off.y / norm_coeff_sibson;
    ipolant.s_pos_off.z = pos_off.z / norm_coeff_sibson;

    ipolant.s_tex_off.u = tex_off.u / norm_coeff_sibson;
    ipolant.s_tex_off.v = tex_off.v / norm_coeff_sibson;

    // check if nni is meaningful
    if(!(norm_coeff_sibson > 0)){
      return false;
    }

    for(it = coor_sibson.begin() ; it != coor_sibson.end() ; ++it, ++c_idx){
      if(!(it->second > 0))
	return false;
    }


#if 0
    glm::vec3 pos_glm(ipolant.s_pos_off.x,ipolant.s_pos_off.y,ipolant.s_pos_off.z);
    const float od = glm::length(pos_glm);
    if(od > 0.1){
      std::cerr << "ERROR: offset_too_large at " << ipolant.s_pos_off << " -> " << od << " norm_coeff_sibson: " << norm_coeff_sibson << " cgal_correct?: " << (int) is_correct_natural_neighborhood(m_dt,ipos, coor_sibson.begin(), coor_sibson.end(), norm_coeff_sibson) << std::endl;
      unsigned nnid = 0;
      for(it = coor_sibson.begin() ; it != coor_sibson.end() ; ++it, ++c_idx){
	double contribution_i = it->second;
	std::cerr << "\t" << nnid << " contribution_i: " << contribution_i << std::endl;
	++nnid;
      }
    }
#endif

    //return true;
    return true;
    //return is_correct_natural_neighborhood(m_dt,ipos, coor_sibson.begin(), coor_sibson.end(), norm_coeff_sibson);
  }
    
