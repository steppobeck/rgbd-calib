#include <fensterchen.hpp>
#include <rgbdsensor.hpp>



#include <fstream>
#include <iostream>


int main(int argc, char* argv[]){

  RGBDConfig cfg;
  
  cfg.size_rgb = glm::uvec2(1280, 1080);
  cfg.size_d   = glm::uvec2(512, 424);

  cfg.principal_rgb = glm::vec2(701.972, 532.143);
  cfg.principal_d   = glm::vec2(257.01, 209.078);

  cfg.focal_rgb = glm::vec2(1030.83, 1030.5);
  cfg.focal_d   = glm::vec2(355.434, 355.672);

  cfg.eye_d_to_eye_rgb[0][0] = 0.999950;
  cfg.eye_d_to_eye_rgb[0][1] = -0.009198;
  cfg.eye_d_to_eye_rgb[0][2] = -0.003908;
  cfg.eye_d_to_eye_rgb[0][3] = 0.0;

  cfg.eye_d_to_eye_rgb[1][0] = 0.009169;
  cfg.eye_d_to_eye_rgb[1][1] = 0.999932;
  cfg.eye_d_to_eye_rgb[1][2] = -0.007234;
  cfg.eye_d_to_eye_rgb[1][3] = 0.0;
 
  cfg.eye_d_to_eye_rgb[2][0] = 0.003974;
  cfg.eye_d_to_eye_rgb[2][1] = 0.007198;
  cfg.eye_d_to_eye_rgb[2][2] = 0.999966;
  cfg.eye_d_to_eye_rgb[2][3] = 0.0;

  cfg.eye_d_to_eye_rgb[3][0] = -0.051237;
  cfg.eye_d_to_eye_rgb[3][1] = 0.000667;
  cfg.eye_d_to_eye_rgb[3][2] = 0.000195;
  cfg.eye_d_to_eye_rgb[3][3] = 1.0;

  cfg.serverport = "141.54.147.27:7000";

  RGBDSensor sensor(cfg);
  unsigned char* rgb = sensor.frame_rgb;
  float* depth = sensor.frame_d;



  Window win(glm::ivec2(800,800), true /*3D mode*/);

  while (!win.shouldClose()) {

    auto t = win.getTime();
    if (win.isKeyPressed(GLFW_KEY_ESCAPE)) {
      win.stop();
    }

    // receive frames
    sensor.recv(false /*recv ir?*/);

    // rotate the 3D reconstruction
    glTranslatef(0.0,0.0,2.0);
    glRotatef(180.0*std::sin(0.1*t)/M_PI,0.0,1.0,0.0);
    glRotatef(180,0.0,1.0,0.0);
    glRotatef(-90,0.0,0.0,1.0);

    glPointSize(1.1);
    glBegin(GL_POINTS);
    // do 3D recosntruction for each depth pixel
    for(unsigned y = 0; y < cfg.size_d.y; ++y){
      for(unsigned x = 0; x < cfg.size_d.x; ++x){

	float d = depth[y* cfg.size_d.x + x];
	glm::vec3 pos3D = sensor.calc_pos_d(x, y, d);
	glm::vec2 pos2D_rgb = sensor.calc_pos_rgb(pos3D);

	// convert from float coordinates to nearest interger coordinates
	const unsigned xc = std::max( 0u, 
				      std::min( cfg.size_rgb.x - 1u, (unsigned) floor(pos2D_rgb.x)));
	const unsigned yc = std::max( 0u,
				      std::min( cfg.size_rgb.y - 1u, (unsigned) floor(pos2D_rgb.y)));
	  
	unsigned char r = rgb[(yc * cfg.size_rgb.x * 3) + 3 * xc];
	unsigned char g = rgb[(yc * cfg.size_rgb.x * 3) + 3 * xc + 1];
	unsigned char b = rgb[(yc * cfg.size_rgb.x * 3) + 3 * xc + 2];

	glColor3f(r*1.0/255, g*1.0/255, b*1.0/255);
	glVertex3f(pos3D.x, pos3D.y, pos3D.z);

      }
    }
    glEnd();

    

    //auto m = win.mousePosition();

    

    win.update();
  }

  return 0;
}