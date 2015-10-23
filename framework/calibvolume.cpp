#include "calibvolume.hpp"

#include <cmath>
#include <fstream>


xyz
CalibVolume::getTrilinear(xyz* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z){

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


uv
CalibVolume::getTrilinear(uv* data, unsigned width, unsigned height, unsigned depth, float x, float y, float z){

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




CalibVolume::CalibVolume(const char* filename_xyz, const char* filename_uv)
  : width(0),
    height(0),
    depth(0),
    min_d(0),
    max_d(0),
    cv_xyz(0),
    cv_uv(0)
{
  // load from files
  FILE* f_xyz = fopen( filename_xyz, "rb");
  fread(&width, sizeof(unsigned), 1, f_xyz);
  fread(&height, sizeof(unsigned), 1, f_xyz);
  fread(&depth, sizeof(unsigned), 1, f_xyz);
  fread(&min_d, sizeof(float), 1, f_xyz);
  fread(&max_d, sizeof(float), 1, f_xyz);
  cv_xyz = new xyz[width * height * depth];
  fread(cv_xyz, sizeof(xyz), width * height * depth, f_xyz);
  fclose(f_xyz);
  
  
  FILE* f = fopen( filename_uv, "rb");
  fread(&width, sizeof(unsigned), 1, f);
  fread(&height, sizeof(unsigned), 1, f);
  fread(&depth, sizeof(unsigned), 1, f);
  fread(&min_d, sizeof(float), 1, f);
  fread(&max_d, sizeof(float), 1, f);
  cv_uv  = new uv [width * height * depth];
  fread(cv_uv, sizeof(uv), width * height * depth, f);
  fclose(f);

}



CalibVolume::CalibVolume(unsigned w, unsigned h, unsigned d, float mi_d, float ma_d)
  : width(w),
    height(h),
    depth(d),
    min_d(mi_d),
    max_d(ma_d),
    cv_xyz(new xyz [width * height * depth]),
    cv_uv(new uv [width * height * depth])
{} 


CalibVolume::~CalibVolume(){
  delete [] cv_xyz;
  delete [] cv_uv;
}


void
CalibVolume::save(const char* filename_xyz, const char* filename_uv){

  {
    FILE* f = fopen( filename_xyz, "wb");
    fwrite(&width, sizeof(unsigned), 1, f);
    fwrite(&height, sizeof(unsigned), 1, f);
    fwrite(&depth, sizeof(unsigned), 1, f);
    fwrite(&min_d, sizeof(float), 1, f);
    fwrite(&max_d, sizeof(float), 1, f);
    fwrite(cv_xyz, sizeof(xyz), width * height * depth, f);
    fclose(f);
  }
  std::cout << "saved " << filename_xyz << std::endl;
  {
    FILE* f = fopen( filename_uv, "wb");
    fwrite(&width, sizeof(unsigned), 1, f);
    fwrite(&height, sizeof(unsigned), 1, f);
    fwrite(&depth, sizeof(unsigned), 1, f);
    fwrite(&min_d, sizeof(float), 1, f);
    fwrite(&max_d, sizeof(float), 1, f);
    fwrite(cv_uv, sizeof(uv), width * height * depth, f);
    fclose(f);
  }
  std::cout << "saved " << filename_uv << std::endl;

}

glm::vec3
CalibVolume::lookupPos3D(float x_norm, float y_norm, float d){
  const float x = width  *  x_norm;
  const float y = height *  y_norm;
  const float z = depth  *  ( d - min_d)/(max_d - min_d);

  xyz pos = getTrilinear(cv_xyz, width, height, depth, x , y , z );
  return glm::vec3(pos.x,pos.y,pos.z);
}

glm::vec2
CalibVolume::lookupPos2D_normalized(float x_norm, float y_norm, float d){
  const float x = width  *  x_norm;
  const float y = height *  y_norm;
  const float z = depth  *  ( d - min_d)/(max_d - min_d);

  uv tex = getTrilinear(cv_uv, width, height, depth, x , y , z );
  return glm::vec2(tex.u,tex.v);
}


#if 0
const float x = cv_width  *  ( sp.tex_depth.u)/ m_calibs[0]->getWidth();
const float y = cv_height *  ( sp.tex_depth.v)/ m_calibs[0]->getHeight();
const float z = cv_depth  *  ( sp.depth - m_cv_min_ds[0])/(m_cv_max_ds[0] - m_cv_min_ds[0]);

xyz pos = getTrilinear(m_cv_xyzs[0], cv_width, cv_height, cv_depth, x , y , z );
uv  tex = getTrilinear(m_cv_uvs[0], cv_width, cv_height, cv_depth, x , y , z );

sp.pos_offset.x = sp.pos_real[0] - pos.x;
sp.pos_offset.y = sp.pos_real[1] - pos.y;
sp.pos_offset.z = sp.pos_real[2] - pos.z;

sp.tex_offset.u = sp.tex_color.u/m_calibs[0]->getWidthC() - tex.u;
sp.tex_offset.v = sp.tex_color.v/m_calibs[0]->getHeightC() - tex.v;
#endif
