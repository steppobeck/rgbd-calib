#include <window.hpp>
#include <calibvolume.hpp>
#include <rgbdsensor.hpp>
#include <CMDParser.hpp>

#include <iostream>
#include <fstream>


glm::vec3 bbx_min = glm::vec3(-10.0, -10.0, -10.0);
glm::vec3 bbx_max = glm::vec3( 10.0, 10.0,  10.0);
  
bool clip(const glm::vec3& p){
  if(p.x < bbx_min.x ||
     p.y < bbx_min.y ||
     p.z < bbx_min.z ||
     p.x > bbx_max.x ||
     p.y > bbx_max.y ||
     p.z > bbx_max.z){
  return true;
}
return false;
}


int main(int argc, char* argv[]){




  CMDParser p("basefilename_cv .... serverport");
  p.addOpt("bbx",6,"bounding_box", "specify the bounding box x_min y_min z_min x_max y_max z_max in meters, default -10.0 -10.0 -10.0 10.0 10.0 10.0");

  p.init(argc,argv);
  if(p.isOptSet("bbx")){
    bbx_min = glm::vec3(p.getOptsFloat("bbx")[0], p.getOptsFloat("bbx")[1], p.getOptsFloat("bbx")[2]);
    bbx_max = glm::vec3(p.getOptsFloat("bbx")[3], p.getOptsFloat("bbx")[4], p.getOptsFloat("bbx")[5]);
    std::cout << "setting bounding box to min: " << bbx_min << " -> max: " << bbx_max << std::endl;
  }


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


    std::vector<glm::vec3> candidate_positions;
    float max_y_level = std::numeric_limits<float>::lowest();

    for(unsigned s_num = 0; s_num < num_streams; ++s_num){
      // do 3D recosntruction for each depth pixel
      for(unsigned y = 0; y < sensor.config.size_d.y; ++y){
	for(unsigned x = 0; x < sensor.config.size_d.x; ++x){
	  const unsigned d_idx = y* sensor.config.size_d.x + x;
	  float d = s_num == 0 ? sensor.frame_d[d_idx] : sensor.slave_frames_d[s_num - 1][d_idx];
	  if(d < cvs[s_num]->min_d || d > cvs[s_num]->max_d)
	    continue;
	  
	  glm::vec3 pos3D = cvs[s_num]->lookupPos3D( x * 1.0/sensor.config.size_d.x,
						     y * 1.0/sensor.config.size_d.y, d);

	  if(clip(pos3D)){
	    continue;
	  }

	  max_y_level = std::max(max_y_level, pos3D.y);

	  candidate_positions.push_back(pos3D);

	}
      }
    }


    const float head_height = 0.3;
    std::vector<glm::vec3> head_candidate_positions;
    for(const auto& p : candidate_positions){
      if(max_y_level - p.y < head_height){
	head_candidate_positions.push_back(p);
      }
    }


    glDisable(GL_DEPTH_TEST);
    glBegin(GL_POINTS);

    glm::vec3 avg(0.0,0.0,0.0);
    for(const auto& p : head_candidate_positions){
      avg.x += p.x;
      avg.y += p.y;
      avg.z += p.z;
      glColor3f(0.0,0.0,1.0);
      glVertex3f(p.x, p.y, p.z);

    }
    avg.x /= head_candidate_positions.size();
    avg.y /= head_candidate_positions.size();
    avg.z /= head_candidate_positions.size();


    glPointSize(5.0);
    glColor3f(1.0,0.0,0.0);
    glVertex3f(avg.x, avg.y, avg.z);
    glEnd();

    win.update();
  }

  return 0;
}




