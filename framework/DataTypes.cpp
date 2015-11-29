#include "DataTypes.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <fstream>

#if 0
  cb.pose_offset[0][0] = 0.999970;
  cb.pose_offset[0][1] = -0.001647;
  cb.pose_offset[0][2] = 0.007582;

  cb.pose_offset[1][0] = 0.001702;
  cb.pose_offset[1][1] = 0.999973;
  cb.pose_offset[1][2] = -0.007256;

  cb.pose_offset[2][0] = -0.007571;
  cb.pose_offset[2][1] = 0.007269;
  cb.pose_offset[2][2] = 0.999944;

  cb.pose_offset[3][0] = -0.003584;
  cb.pose_offset[3][1] = 0.002038;
  cb.pose_offset[3][2] = 0.007816;
#endif

bool
Checkerboard::save_pose_offset(const char* filename){
  std::ofstream off(filename, std::ofstream::binary);
  off.write((const char*) glm::value_ptr(pose_offset), sizeof(glm::mat4));
  off.close();
  std::cout << "Checkerboard:: saving pose offset " << pose_offset << std::endl;
  return true;
}

bool
Checkerboard::load_pose_offset(const char* filename){
  std::ifstream iff(filename, std::ifstream::binary);
  iff.read((char*) glm::value_ptr(pose_offset), sizeof(glm::mat4));
  iff.close();
  std::cout << "Checkerboard:: loaded pose offset " << pose_offset << std::endl;
  return true;
}


std::ostream& operator << (std::ostream& o, const glm::vec3& v){
    o << "(" << v.x << "," << v.y << "," << v.z << ")";
    return o;
  }

std::ostream& operator<< (std::ostream& os, const glm::mat4& m){
  os << "mat4[" << std::fixed << std::endl;
  os << "       ("               << m[0][0] << ", " << m[1][0] << ", " << m[2][0]  << ", " << m[3][0] << ")," << std::endl;
  os << "       ("               << m[0][1] << ", " << m[1][1] << ", " << m[2][1]  << ", " << m[3][1] << ")," << std::endl;
  os << "       ("               << m[0][2] << ", " << m[1][2] << ", " << m[2][2]  << ", " << m[3][2] << ")," << std::endl;
  os << "       ("               << m[0][3] << ", " << m[1][3] << ", " << m[2][3]  << ", " << m[3][3] << ") ]" << std::endl;

  return os;
}


  uv interpolate(const uv& a, const uv& b, float t){
    uv r;
    r.u = (1.0f - t)*a.u + t*b.u;
    r.v = (1.0f - t)*a.v + t*b.v;
    return r;
  }



  xyz interpolate(const xyz& a, const xyz& b, float t){
    xyz r;
    r.x = (1.0f - t)*a.x + t*b.x;
    r.y = (1.0f - t)*a.y + t*b.y;
    r.z = (1.0f - t)*a.z + t*b.z;
    return r;
  }



  /*extern*/
  xyz
  operator* (const float v, const xyz& b){
    xyz res;
    res.x = v * b.x;
    res.y = v * b.y;
    res.z = v * b.z;
    return res;
  }


  /*extern*/
  uv
  operator* (const float v, const uv& b){
    uv res;
    res.u = v * b.u;
    res.v = v * b.v;
    return res;
  }



  /*extern*/
  xyz
  operator+ (const xyz& a, const xyz& b){
    xyz res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
  }

  std::ostream& operator << (std::ostream& o, const xyz& v){
    o << "(" << v.x << "," << v.y << "," << v.z << ")";
    return o;
  }

  /*extern*/
  uv
  operator+ (const uv& a, const uv& b){
    uv res;
    res.u = a.u + b.u;
    res.v = a.v + b.v;
    return res;
  }


  /*extern*/
  uv
  operator- (const uv& a, const uv& b){
    uv res;
    res.u = a.u - b.u;
    res.v = a.v - b.v;
    return res;
  }

  std::ostream& operator << (std::ostream& o, const uv& v){
    o << "(" << v.u << "," << v.v << ")";
    return o;
  }

  /*extern*/
  xyz_d
  operator* (const float v, const xyz_d& b){
    xyz_d res;
    res.x = v * b.x;
    res.y = v * b.y;
    res.z = v * b.z;
    return res;
  }


  /*extern*/
  uv_d
  operator* (const float v, const uv_d& b){
    uv_d res;
    res.u = v * b.u;
    res.v = v * b.v;
    return res;
  }



  /*extern*/
  xyz_d
  operator+ (const xyz_d& a, const xyz& b){
    xyz_d res;
    res.x = a.x + b.x;
    res.y = a.y + b.y;
    res.z = a.z + b.z;
    return res;
  }


  /*extern*/
  uv_d
  operator+ (const uv_d& a, const uv& b){
    uv_d res;
    res.u = a.u + b.u;
    res.v = a.v + b.v;
    return res;
  }



  CandidateSample::CandidateSample(const float w, const xyz& p_off, const uv& t_off):
    weight(w),
    pos_off(),
    tex_off()
  {
    pos_off = p_off;
    tex_off = t_off;
  }

  CandidateSample::~CandidateSample()
  {}


  /*extern*/
  bool
  operator < (const CandidateSample& a, const CandidateSample& b){
    return a.weight > b.weight;
  }






  /* extern */
  std::ostream&
  operator<< (std::ostream& os, const samplePoint& a)
  {
    os << "samplePoint( " << std::fixed 
       << " depth " << a.depth
       << " tex_color.u " << a.tex_color.u
       << " tex_color.v " << a.tex_color.v
       << " tex_depth.u " << a.tex_depth.u
       << " tex_depth.v " << a.tex_depth.v
       << " pos_offset.x " << a.pos_offset.x
       << " pos_offset.y " << a.pos_offset.y
       << " pos_offset.z " << a.pos_offset.z
       << " tex_offset.u " << a.tex_offset.u
       << " tex_offset.v " << a.tex_offset.v
       << " pos_real " << a.pos_real
       << " quality "  << a.quality
       << ")";


    return os;
  }



  float
  getBilinear(float* data, unsigned width, unsigned height, float x, float y){


    // calculate weights and boundaries along x direction
    unsigned xa = std::floor(x);
    unsigned xb = std::ceil(x);
    float w_xb = x - xa;
    float w_xa = 1.0 - w_xb;

    // calculate weights and boundaries along y direction
    unsigned ya = std::floor(y);
    unsigned yb = std::ceil(y);
    float w_yb = y - ya;
    float w_ya = 1.0 - w_yb;

    // calculate indices to access data
    const unsigned idmax = width * height;
    unsigned id00 = std::min( ya * width + xa  , idmax);
    unsigned id10 = std::min( ya * width + xb  , idmax);
    unsigned id01 = std::min( yb * width + xa  , idmax);
    unsigned id11 = std::min( yb * width + xb  , idmax);

    // 1. interpolate between x direction;
    float tmp_ya = w_xa * data[id00] + w_xb * data[id10];
    float tmp_yb = w_xa * data[id01] + w_xb * data[id11];
    // 2. interpolate between y direction;
    float result = w_ya * tmp_ya + w_yb * tmp_yb;
    return result;
  }




xyz getTrilinear(xyz* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z){

    // calculate weights and boundaries along x direction
    unsigned xa = std::floor(x);
    unsigned xb = std::ceil(x);
    float w_xb = x - xa;
    float w_xa = 1.0 - w_xb;

    // calculate weights and boundaries along y direction
    unsigned ya = std::floor(y);
    unsigned yb = std::ceil(y);
    float w_yb = y - ya;
    float w_ya = 1.0 - w_yb;

    // calculate weights and boundaries along z direction
    unsigned za = std::floor(z);
    unsigned zb = std::ceil(z);
    float w_zb = z - za;
    float w_za = 1.0 - w_zb;

    // calculate indices to access data
    const unsigned idmax = width * height * depth;
    unsigned id000 = std::min( za * width * height + ya * width + xa  , idmax);
    unsigned id100 = std::min( za * width * height + ya * width + xb  , idmax);
    unsigned id110 = std::min( za * width * height + yb * width + xb  , idmax);
    unsigned id010 = std::min( za * width * height + yb * width + xa  , idmax);

    unsigned id001 = std::min( zb * width * height + ya * width + xa  , idmax);
    unsigned id101 = std::min( zb * width * height + ya * width + xb  , idmax);
    unsigned id111 = std::min( zb * width * height + yb * width + xb  , idmax);
    unsigned id011 = std::min( zb * width * height + yb * width + xa  , idmax);



    // 1. interpolate between x direction: 4 times;
    xyz   tmp_000_100 = w_xa * data[id000] + w_xb * data[id100];
    xyz   tmp_010_110 = w_xa * data[id010] + w_xb * data[id110];
    xyz   tmp_001_101 = w_xa * data[id001] + w_xb * data[id101];
    xyz   tmp_011_111 = w_xa * data[id011] + w_xb * data[id111];

    // 2. interpolate between y direction: 2 times;

    xyz   tmp_A = w_ya * tmp_000_100 + w_yb * tmp_010_110;
    xyz   tmp_B = w_ya * tmp_001_101 + w_yb * tmp_011_111;

    xyz result = w_za * tmp_A + w_zb * tmp_B;

    return result;
}


uv getTrilinear(uv* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z){

    // calculate weights and boundaries along x direction
    unsigned xa = std::floor(x);
    unsigned xb = std::ceil(x);
    float w_xb = x - xa;
    float w_xa = 1.0 - w_xb;

    // calculate weights and boundaries along y direction
    unsigned ya = std::floor(y);
    unsigned yb = std::ceil(y);
    float w_yb = y - ya;
    float w_ya = 1.0 - w_yb;

    // calculate weights and boundaries along z direction
    unsigned za = std::floor(z);
    unsigned zb = std::ceil(z);
    float w_zb = z - za;
    float w_za = 1.0 - w_zb;

    // calculate indices to access data
    const unsigned idmax = width * height * depth;
    unsigned id000 = std::min( za * width * height + ya * width + xa  , idmax);
    unsigned id100 = std::min( za * width * height + ya * width + xb  , idmax);
    unsigned id110 = std::min( za * width * height + yb * width + xb  , idmax);
    unsigned id010 = std::min( za * width * height + yb * width + xa  , idmax);

    unsigned id001 = std::min( zb * width * height + ya * width + xa  , idmax);
    unsigned id101 = std::min( zb * width * height + ya * width + xb  , idmax);
    unsigned id111 = std::min( zb * width * height + yb * width + xb  , idmax);
    unsigned id011 = std::min( zb * width * height + yb * width + xa  , idmax);



    // 1. interpolate between x direction: 4 times;
    uv   tmp_000_100 = w_xa * data[id000] + w_xb * data[id100];
    uv   tmp_010_110 = w_xa * data[id010] + w_xb * data[id110];
    uv   tmp_001_101 = w_xa * data[id001] + w_xb * data[id101];
    uv   tmp_011_111 = w_xa * data[id011] + w_xb * data[id111];

    // 2. interpolate between y direction: 2 times;

    uv   tmp_A = w_ya * tmp_000_100 + w_yb * tmp_010_110;
    uv   tmp_B = w_ya * tmp_001_101 + w_yb * tmp_011_111;

    uv result = w_za * tmp_A + w_zb * tmp_B;

    return result;
}


glm::vec3 calcMean(const std::vector<glm::vec3>& vecs){

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  for(const auto& v : vecs){
    x += v[0];
    y += v[1];
    z += v[2];
  }
  return glm::vec3(x/vecs.size(), y/vecs.size(), z/vecs.size());
}


