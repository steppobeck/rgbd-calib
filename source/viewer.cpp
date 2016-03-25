#include <window.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <CMDParser.hpp>

#include <iostream>
#include <fstream>

int main(int argc, char* argv[]){

  CMDParser p("basefilename_cv .... serverport");
  p.init(argc,argv);


  const unsigned num_streams(p.getArgs().size() - 1);
  std::vector<CalibVolume*> cvs;
  for(unsigned i = 0; i < num_streams; ++i){
    std::string basefilename = p.getArgs()[i];
    std::string filename_xyz(basefilename + "_xyz");
    std::string filename_uv(basefilename + "_uv");
    cvs.push_back(new CalibVolume(filename_xyz.c_str(), filename_uv.c_str()));
  }


  RGBDConfig cfg;
  cfg.serverport = p.getArgs()[num_streams];
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);

  RGBDSensor sensor(cfg, num_streams - 1);


  Window win(glm::ivec2(800,800), true /*3D mode*/);


  while (!win.shouldClose()) {

    auto t = win.getTime();
    if (win.isKeyPressed(GLFW_KEY_ESCAPE)) {
      win.stop();
    }


    // receive frames
    sensor.recv(false /*do not recv ir!*/);
    sensor.display_rgb_d();

    glPointSize(1.0);
    glBegin(GL_POINTS);

    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      // do 3D recosntruction for each depth pixel
      for(unsigned y = 0; y < sensor.config.size_d.y; ++y){
	for(unsigned x = 0; x < sensor.config.size_d.x; ++x){
	  const unsigned d_idx = y* sensor.config.size_d.x + x;
	  float d = s_num == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[s_num - 1][d_idx];
	  if(d < cvs[s_num]->min_d || d > cvs[s_num]->max_d)
	    continue;
	  
	  glm::vec3 pos3D;
	  glm::vec2 pos2D_rgb;
	  
	  pos3D = cvs[s_num]->lookupPos3D( x * 1.0/sensor.config.size_d.x,
					   y * 1.0/sensor.config.size_d.y, d);
	  glm::vec2 pos2D_rgb_norm = cvs[s_num]->lookupPos2D_normalized( x * 1.0/sensor.config.size_d.x, 
									 y * 1.0/sensor.config.size_d.y, d);
	  pos2D_rgb = glm::vec2(pos2D_rgb_norm.x * sensor.config.size_rgb.x,
				pos2D_rgb_norm.y * sensor.config.size_rgb.y);
	  
	  glm::vec3 rgb = sensor.get_rgb_bilinear_normalized(pos2D_rgb, s_num);
	  glColor3f(rgb.x, rgb.y, rgb.z);
	  glVertex3f(pos3D.x, pos3D.y, pos3D.z);

	}
      }
    }
    glEnd();

    win.update();
  }

  return 0;
}

