#include <CMDParser.hpp>
#include <LPCDataTypes.hpp>
#include <LPCNearestNeighbourSearch.hpp>


#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <climits>
#include <limits>
#include <cfloat>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdlib>

#define DEFAULT_PRECISION 15


class lpc{
public:
  lpc();
  bool read(const std::string& filename);
  bool write(const std::string& filename);
  void translate(const glm::dvec3& t);
  bool rotate(const glm::dmat4& r);
  bool crop_to(size_t c);
  bool cut_out(size_t c);
  bool distort(size_t c);
  glm::dvec3 bbx_min;
  glm::dvec3 bbx_max;
  std::vector<surfel> surfels;
  bool has_all;
private:
  void update_bbx();
};


lpc::lpc()
  : surfels(),
    has_all(false)
{}

bool
lpc::read(const std::string& filename){
  surfels.clear();
  has_all = filename.substr(filename.find_last_of(".") + 1) == "xyz_all" ? true : false;

  std::cout << "lpc::read() parsing file: " << filename << " -> has_all: " << int(has_all) << std::endl;
  std::ifstream lpc_ascii(filename.c_str());
  std::string surfel_ascii;


  bbx_min[0] = std::numeric_limits<double>::max();
  bbx_min[1] = std::numeric_limits<double>::max();
  bbx_min[2] = std::numeric_limits<double>::max();
  bbx_max[0] = std::numeric_limits<double>::lowest(); 
  bbx_max[1] = std::numeric_limits<double>::lowest();
  bbx_max[2] = std::numeric_limits<double>::lowest();

  while(std::getline(lpc_ascii, surfel_ascii)){

    std::istringstream lineparser(surfel_ascii);
    surfel s;

    // parse surfel ascii
    lineparser >> std::setprecision(DEFAULT_PRECISION) >> s.p[0];
    lineparser >> std::setprecision(DEFAULT_PRECISION) >> s.p[1];
    lineparser >> std::setprecision(DEFAULT_PRECISION) >> s.p[2];
    if(has_all){
      lineparser >> s.n[0];
      lineparser >> s.n[1];
      lineparser >> s.n[2];
    }
    lineparser >> s.c[0];
    lineparser >> s.c[1];
    lineparser >> s.c[2];
    if(has_all){
      lineparser >> s.r;
    }

    // update bounding box
    bbx_min[0] = std::min(bbx_min[0], s.p[0]);
    bbx_min[1] = std::min(bbx_min[1], s.p[1]);
    bbx_min[2] = std::min(bbx_min[2], s.p[2]);

    bbx_max[0] = std::max(bbx_max[0], s.p[0]);
    bbx_max[1] = std::max(bbx_max[1], s.p[1]);
    bbx_max[2] = std::max(bbx_max[2], s.p[2]);
    
    surfels.push_back(s);
			
  }
  lpc_ascii.close();
  return true;
}


bool
lpc::write(const std::string& filename){

  has_all = filename.substr(filename.find_last_of(".") + 1) == "xyz_all" ? true : false;

  std::cout << "lpc::write() writing file: " << filename << " -> has_all: " << int(has_all) << std::endl;
  std::ofstream lpc_ascii(filename.c_str());
  
  for(const auto& s : surfels){

    std::stringstream lpc_ascii_s;
    lpc_ascii_s << std::setprecision(DEFAULT_PRECISION) << s.p[0] << " ";
    lpc_ascii_s << std::setprecision(DEFAULT_PRECISION) << s.p[1] << " ";
    lpc_ascii_s << std::setprecision(DEFAULT_PRECISION) << s.p[2] << " ";
    if(has_all){
      lpc_ascii_s << s.n[0] << " ";
      lpc_ascii_s << s.n[1] << " ";
      lpc_ascii_s << s.n[2] << " ";
    }
    lpc_ascii_s << s.c[0] << " ";
    lpc_ascii_s << s.c[1] << " ";
    lpc_ascii_s << s.c[2];
    if(has_all){
      lpc_ascii_s << " " << s.r;
    }
    lpc_ascii_s << std::endl;

    lpc_ascii << lpc_ascii_s.str();
  }
  
  lpc_ascii.close();
  return true;
}

void
lpc::translate(const glm::dvec3& t){
  for(auto& s : surfels){
    s.p[0] += t[0];
    s.p[1] += t[1];
    s.p[2] += t[2];
  }
}

bool
lpc::rotate(const glm::dmat4& r){
  LPCNearestNeighbourSearch nns(&surfels);
  surfel center_dummy_surfel;
  center_dummy_surfel.p = 0.5 * (bbx_min + bbx_max);
  std::vector<surfel> n = nns.search(center_dummy_surfel, 1);
  if(n.size() != 1){
    std::cout << "ERROR: lpc::crop_to() -> could not find center surfel!" << std::endl;
    return false;
  }
  for(auto& s : surfels){
    glm::dvec4 p;
    p[0] = s.p[0] - n[0].p[0];
    p[1] = s.p[1] - n[0].p[1];
    p[2] = s.p[2] - n[0].p[2];
    p[3] = 1.0;

    p = r * p;

    s.p[0] = p[0] + n[0].p[0];
    s.p[1] = p[1] + n[0].p[1];
    s.p[2] = p[2] + n[0].p[2];

  }
  return true;
}

bool
lpc::crop_to(size_t count){
  LPCNearestNeighbourSearch nns(&surfels);
  surfel center_dummy_surfel;
  center_dummy_surfel.p = 0.5 * (bbx_min + bbx_max);
  std::vector<surfel> n = nns.search(center_dummy_surfel, 1);
  if(n.size() != 1){
    std::cout << "ERROR: lpc::crop_to() -> could not find center surfel!" << std::endl;
    return false;
  }
  std::vector<surfel> cropped_cloud = nns.search(n[0], std::min(count, surfels.size()));
  surfels = cropped_cloud;
  update_bbx();
  return true;
}

bool
lpc::cut_out(size_t count){
  LPCNearestNeighbourSearch nns(&surfels);
  surfel center_dummy_surfel;
  center_dummy_surfel.p = 0.5 * (bbx_min + bbx_max);
  std::vector<surfel> n = nns.search(center_dummy_surfel, 1);
  if(n.size() != 1){
    std::cout << "ERROR: lpc::crop_to() -> could not find center surfel!" << std::endl;
    return false;
  }
  std::vector<size_t> nids = nns.search_id(n[0], std::min(count, surfels.size()));
  std::vector<surfel> res;
  for(unsigned idx = 0; idx != surfels.size(); ++idx){
    if(std::find(nids.begin(), nids.end(), idx) == nids.end()){
      res.push_back(surfels[idx]);
    }
  }
  surfels = res;
  update_bbx();
  return true;
}


bool
lpc::distort(size_t count){
  LPCNearestNeighbourSearch nns(&surfels);
  surfel center_dummy_surfel;
  center_dummy_surfel.p = 0.5 * (bbx_min + bbx_max);
  std::vector<surfel> n = nns.search(center_dummy_surfel, 1);
  if(n.size() != 1){
    std::cout << "ERROR: lpc::crop_to() -> could not find center surfel!" << std::endl;
    return false;
  }
  std::vector<size_t> nids = nns.search_id(n[0], std::min(count, surfels.size()));
  float max_d = 0;
  for(unsigned idx = 0; idx != nids.size(); ++idx){
    const float d = glm::length(n[0].p - surfels[nids[idx]].p);
    max_d = std::max(d, max_d);
  }

  std::vector<surfel> res;
  for(unsigned idx = 0; idx != surfels.size(); ++idx){
    if(std::find(nids.begin(), nids.end(), idx) == nids.end()){
      res.push_back(surfels[idx]);
    }
    else{
      const float d = glm::length(n[0].p - surfels[idx].p);
      surfels[idx].p[0] += (1.0 - (d/max_d)) * 0.05;
      surfels[idx].p[1] += (1.0 - (d/max_d)) * 0.05;
      surfels[idx].p[2] += (1.0 - (d/max_d)) * 0.05;
    }
  }
  surfels = res;
  update_bbx();
  return true;
}


void
lpc::update_bbx(){
  double bbx_minx = std::numeric_limits<double>::max();
  double bbx_miny = std::numeric_limits<double>::max();
  double bbx_minz = std::numeric_limits<double>::max();
  double bbx_maxx = std::numeric_limits<double>::lowest();
  double bbx_maxy = std::numeric_limits<double>::lowest();
  double bbx_maxz = std::numeric_limits<double>::lowest();

#pragma omp parallel for reduction (min : bbx_minx, bbx_miny, bbx_minz) reduction (max : bbx_maxx, bbx_maxy, bbx_maxz)
  for(unsigned idx = 0; idx < surfels.size(); ++idx){
    // update bounding box
    bbx_minx = std::min(bbx_minx, surfels[idx].p[0]);
    bbx_miny = std::min(bbx_miny, surfels[idx].p[1]);
    bbx_minz = std::min(bbx_minz, surfels[idx].p[2]);
    
    bbx_maxx = std::max(bbx_maxx, surfels[idx].p[0]);
    bbx_maxy = std::max(bbx_maxy, surfels[idx].p[1]);
    bbx_maxz = std::max(bbx_maxz, surfels[idx].p[2]);
  }
  bbx_min[0] = bbx_minx;
  bbx_min[1] = bbx_miny;
  bbx_min[2] = bbx_minz;
  bbx_max[0] = bbx_maxx;
  bbx_max[1] = bbx_maxy;
  bbx_max[2] = bbx_maxz;
}


int main(int argc, char* argv[]){

  std::string outfilename;
  bool has_translation = false;
  glm::dvec3 translation;

  bool has_rotation = false;
  glm::dmat4  rotation;
  glm::dvec3 rotation_center;

  bool normals_as_strokes = false;
  float percentage = 0.0;

  size_t crop_target = 0;
  size_t cut_target = 0;
  size_t distort_target = 0;
  CMDParser p("cloudA cloudB");
  p.addOpt("o",1,"outfilename", "specify the filename of the output");
  p.addOpt("t",3,"translate", "specify the translation (x, y, z) of cloudB");
  p.addOpt("r",3,"rotate", "specify the rotation in degree of cloudB around x-axis y-axis z-axis");
  p.addOpt("s",-1,"strokes", "write stroke vectors as surfel normals");
  p.addOpt("p",1,"percentage", "override color from p percentage (default: 0.0) 0.0 -> 1.0");
  p.addOpt("c",1,"crop", "crop cloudA starting around center of boundingbox to x surfels");
  p.addOpt("x",1,"xcut", "cut x surfels from cloudA starting around center of boundingbox");
  p.addOpt("d",1,"distort", "distort x surfels from cloudA starting around center of boundingbox");
  p.init(argc,argv);

  if(p.isOptSet("o")){
    outfilename = p.getOptsString("o")[0];
  }

  if(p.isOptSet("c")){
    crop_target = p.getOptsInt("c")[0];
    std::cout << "setting crop target to " << crop_target << " surfels." << std::endl;
  }

  if(p.isOptSet("x")){
    cut_target = p.getOptsInt("x")[0];
    std::cout << "setting cut target to " << cut_target << " surfels." << std::endl;
  }

  if(p.isOptSet("d")){
    distort_target = p.getOptsInt("d")[0];
    std::cout << "setting distort target to " << distort_target << " surfels." << std::endl;
  }

  if(p.isOptSet("s")){
    normals_as_strokes = true;
    std::cout << "writing stroke vectors as surfel normals." << std::endl;
  }

  if(p.isOptSet("p")){
    percentage = std::min(1.0f, p.getOptsFloat("p")[0]);
    std::cout << "overwriting percentage: " << percentage << std::endl;
  }

  if(p.isOptSet("r")){
    has_rotation = true;
    glm::mat4 rotX = glm::rotate(glm::mat4(1.0f), glm::radians(p.getOptsFloat("r")[0]), glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat4 rotY = glm::rotate(glm::mat4(1.0f), glm::radians(p.getOptsFloat("r")[1]), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 rotZ = glm::rotate(glm::mat4(1.0f), glm::radians(p.getOptsFloat("r")[2]), glm::vec3(0.0f, 0.0f, 1.0f));
    rotation = rotZ * rotY * rotX;
    std::cout << "rotation of cloudB " << p.getOptsFloat("r")[0] << " (x-axis), "
	      << p.getOptsFloat("r")[1] << " (y-axis), "
	      << p.getOptsFloat("r")[2] << " (z-axis)" << std::endl;
  }

  if(p.isOptSet("t")){
    has_translation = true;
    translation[0] = p.getOptsFloat("t")[0];
    translation[1] = p.getOptsFloat("t")[1];
    translation[2] = p.getOptsFloat("t")[2];
    std::cout << "translation of cloudB by: " << translation << std::endl;
  }

  

  std::vector<lpc*> clouds;
  for(const auto& filename : p.getArgs()){
    lpc* c = new lpc;
    c->read(filename);
    std::cout << "bounding box: " << c->bbx_min << " -> " << c->bbx_max << std::endl;
    clouds.push_back(c);
  }

  const unsigned manipulated_cloud_id = clouds.size() == 2 ? 1 : 0;

  if(crop_target > 0){
    bool success = clouds[manipulated_cloud_id]->crop_to(crop_target);
    if(!success){
      return 1;
    }
    std::cout << "cropped cloudA to bounding box: " << clouds[manipulated_cloud_id]->bbx_min << " -> " << clouds[manipulated_cloud_id]->bbx_max << std::endl;
  }

  if(cut_target > 0){
    bool success = clouds[manipulated_cloud_id]->cut_out(cut_target);
    if(!success){
      return 1;
    }
    std::cout << "cutted "<< cut_target << " surfels from cloudA. result bounding box: " << clouds[manipulated_cloud_id]->bbx_min << " -> " << clouds[manipulated_cloud_id]->bbx_max << std::endl;
  }

  if(distort_target > 0){
    bool success = clouds[manipulated_cloud_id]->distort(distort_target);
    if(!success){
      return 1;
    }
    std::cout << "distorted "<< distort_target << " surfels from cloudA. result bounding box: " << clouds[manipulated_cloud_id]->bbx_min << " -> " << clouds[manipulated_cloud_id]->bbx_max << std::endl;
  }

  if(has_rotation){
    clouds[manipulated_cloud_id]->rotate(rotation);
  }
  if(has_translation){
    clouds[manipulated_cloud_id]->translate(translation);
  }


  if(clouds.size() == 2){
    
    std::cout << "building nearest neighbour search tree for cloudA..." << std::endl;
    LPCNearestNeighbourSearch sA(&clouds[0]->surfels);

    std::cout << "comparing cloudB with cloudA..." << std::endl;
    float max_d = 0.0;
    std::vector<glm::vec4> vec_B_A;
    for(const auto& s : clouds[manipulated_cloud_id]->surfels){
      std::vector<surfel> n = sA.search(s, 1);
      if(n.size() != 1){
	std::cout << "could not find neighbor for surfel in cloudB at position " << s.p << std::endl;
	vec_B_A.push_back(glm::vec4(std::numeric_limits<float>::max()));
      }
      else{
	const glm::vec3 v(glm::vec3(n[0].p[0] - s.p[0], n[0].p[1] - s.p[1], n[0].p[2] - s.p[2]));
	const float d = glm::length(v);
	max_d = std::max(max_d, d);
	vec_B_A.push_back(glm::vec4(v[0],v[1],v[2],d));
      }
    }

    std::cout << "distances are between 0.0 and " << max_d << std::endl;
    for(unsigned idx = 0; idx != clouds[manipulated_cloud_id]->surfels.size(); ++idx){
      const float d = vec_B_A[idx][3];
      if( (d / max_d) > percentage || (0.0 == percentage)){
	clouds[manipulated_cloud_id]->surfels[idx].c = DataPointToColor(d, 0.0, max_d);
      }
      if(normals_as_strokes){
	clouds[manipulated_cloud_id]->surfels[idx].n = glm::vec3(vec_B_A[idx][0], vec_B_A[idx][1], vec_B_A[idx][2]);
      }
    }
  }

  if("" != outfilename){
    clouds[manipulated_cloud_id]->write(outfilename);
  }

  for(auto c : clouds){
    delete c;
  }
  return 0;
}
