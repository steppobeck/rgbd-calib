#ifndef RGBD_CALIB_DATATYPES_HPP
#define RGBD_CALIB_DATATYPES_HPP



#include <glm/vec3.hpp>

#include <iostream>

extern std::ostream& operator << (std::ostream& o, const glm::vec3& v);

  class xyz{
  public:
    float x;
    float y;
    float z;
  };

  extern std::ostream& operator << (std::ostream& o, const xyz& v);
  extern xyz interpolate(const xyz& a, const xyz& b, float t);

  class uv{
  public:
    float u;
    float v;
  };

  extern std::ostream& operator << (std::ostream& o, const uv& v);
  extern uv interpolate(const uv& a, const uv& b, float t);

  class xyz_d{
  public:
    double x;
    double y;
    double z;
  };

  class uv_d{
  public:
    double u;
    double v;
  };


extern xyz operator* (const float, const xyz&);
extern uv operator* (const float, const uv&);


extern xyz operator+ (const xyz&, const xyz&);
extern uv operator+ (const uv&, const uv&);
extern uv operator- (const uv&, const uv&);



extern xyz_d operator* (const float, const xyz_d&);
extern uv_d operator* (const float, const uv_d&);


extern xyz_d operator+ (const xyz_d&, const xyz&);
extern uv_d operator+ (const uv_d&, const uv&);


 class samplePoint{
 public:
   float depth;
   uv tex_color;
   uv tex_depth;
   
   xyz pos_offset;
   uv tex_offset;
   glm::vec3 pos_real;
   float quality;
 };

 /// ostream operator
   extern std::ostream& operator<< (std::ostream&, const samplePoint&);

 class CandidateSample{

 public:
   CandidateSample(const float w, const xyz& p_off, const uv& t_off);
   ~CandidateSample();

   float weight;
   xyz pos_off;
   uv tex_off;

 };

 extern bool operator < (const CandidateSample& a, const CandidateSample& b);


 class evaly{

 public:
   glm::vec3 posV;
   double err_3D;
   double err_2D;
   double distN;
   bool vv;
   unsigned bid;
 };


 class board_desc{
 public:
   float id_pos_x;
   float id_pos_y;
   unsigned bid;
 };




#endif // #ifndef RGBD_CALIB_DATATYPES_HPP

