#include "calibvolume.hpp"

#include <cmath>
#include <fstream>




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
