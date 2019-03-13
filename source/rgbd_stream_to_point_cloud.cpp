#include <CMDParser.hpp>
#include <zmq.hpp>

#include <rgbdsensor.hpp>
#include <window.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string.h>


namespace{
unsigned char*
convertTo8Bit(float* in, unsigned w, unsigned h){
  static unsigned char* out = new unsigned char [w*h];
  static const float norm = 1.0/10.0/*max_distance is 5.0 meter*/;
  for(unsigned idx = 0; idx != w*h; ++idx){
    const float v = norm * in[idx];
    out[idx] = (unsigned char) (255.0 * v);
  }
  return out;
}

unsigned char*
convertRGB2BGR(unsigned char* in, unsigned w, unsigned h){
  static unsigned char* out = new unsigned char [w*h*3];
  unsigned idx = 0;
  for(unsigned y = 0; y != h; ++y){
    for(unsigned x = 0; x != w; ++x){
      const unsigned char b = in[idx + 0];
      const unsigned char g = in[idx + 1];
      const unsigned char r = in[idx + 2];
      out[idx + 0] = r;
      out[idx + 1] = g;
      out[idx + 2] = b;
      ++idx;
      ++idx;
      ++idx;
    }
  }
  return out;
}
}

bool endsWith(const std::string &mainStr, const std::string &toMatch)
{
  if(mainStr.size() >= toMatch.size() &&
      mainStr.compare(mainStr.size() - toMatch.size(), toMatch.size(), toMatch) == 0)
      return true;
    else
      return false;
}

int main(int argc, char* argv[]){

  unsigned width_dir = 512;
  unsigned height_dir = 424;
  unsigned width_c = 1280;
  unsigned height_c = 1080;

  unsigned colorsizebyte = 0;
  unsigned depthsizebyte = 0;
  unsigned irsizebyte = 0;

  bool rgb_is_compressed = false;


  std::vector<std::string> yml_files;

  for(int i = 1; i < argc; ++i) {
    if(endsWith(argv[i], "yml")) {
      yml_files.push_back(argv[i]);

      std::cout << "Found yml file: " << argv[i] << "\n";
    }
  }

  CMDParser p("x");

  p.addOpt("k",1,"num_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.addOpt("a",1,"num_active_cameras", "only show point clouds for num_active_cameras out of num_cameras available streams, default: 1");
  p.addOpt("r",4,"realsense", "enable display for realsense cameras and specify resolution of color and depth sensor e.g. 1280 720 1280 720, default: Kinect V2");
  p.addOpt("s",1,"serverport", "Serverport to receive stream from");


  p.addOpt("d",2,"display_size", "Size of the window, default: 800 800");
  p.addOpt("c",1,"cut_off_threshold", "Distance Cut-Off Threshold in meters, default 10.0");
  p.addOpt("m",1,"mix_in_factor", "Factor for mixing depth visualization with normal vis, default: 0.0");
  p.addOpt("g",1,"gradient_cut_off_threshold", "Gradient cut off threshold, default: FLT_MAX");

  p.init(argc,argv);

  unsigned num_streams = 0;
  unsigned num_active_cameras = 0;

  if(p.isOptSet("k")){
    num_streams = (p.getOptsInt("k")[0]);
    num_active_cameras = num_streams;
  } else {
    std::cout << "k parameter required\n";
    return 1;
  }

  if(p.isOptSet("a")){
    num_active_cameras = (p.getOptsInt("a")[0]);
  }

  float distance_cut_off_threshold = 10.0;
  if(p.isOptSet("c")){
    distance_cut_off_threshold = (p.getOptsFloat("c")[0]);
  }

  float mix_in_factor = 0.0;
  if(p.isOptSet("m")){
    mix_in_factor = (p.getOptsFloat("m")[0]);
  }

  float gradient_cut_off_threshold = FLT_MAX;

  if(p.isOptSet("g")){
    gradient_cut_off_threshold = (p.getOptsFloat("g")[0]);
  }

  glm::ivec2 window_size = glm::ivec2(800, 800);

  if(p.isOptSet("d")){
    window_size[0] = (p.getOptsFloat("d")[0]);
    window_size[1] = (p.getOptsFloat("d")[1]);
  }

  if(p.isOptSet("r")){

    std::cout << "realsense cameras enabled!" << std::endl;

    width_dir = p.getOptsInt("r")[2];
    height_dir = p.getOptsInt("r")[3];
    width_c = p.getOptsInt("r")[0];
    height_c = p.getOptsInt("r")[1];

    if(rgb_is_compressed){
	std::cout << "compressed color not supported for the resolution specified. Exiting" << std::endl;
	exit(0);
    }
    colorsizebyte = rgb_is_compressed ? 460800 : width_c * height_c * 3;
    depthsizebyte = width_dir * height_dir * sizeof(float);

  }
  else{
    colorsizebyte = rgb_is_compressed ? 691200 : width_c * height_c * 3;
    depthsizebyte = width_dir * height_dir * sizeof(float);
  }
  
  if(p.isOptSet("i")){
    irsizebyte = width_dir * height_dir;
  }


  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM,&hwm, sizeof(hwm));
  std::string endpoint(std::string("tcp://") +  p.getOptsString("s")[0] );
  socket.connect(endpoint.c_str());

  std::vector<RGBDConfig> sensor_configs(num_streams);

  for(int sensor_idx = 0; sensor_idx < sensor_configs.size(); ++sensor_idx) {
    sensor_configs[sensor_idx].read(yml_files[sensor_idx].c_str());
  }

  Window win(window_size, true /*3D mode*/);

  while (!win.shouldClose()) {

    auto t = win.getTime();
    if (win.isKeyPressed(GLFW_KEY_ESCAPE)) {
      win.stop();
    }

    zmq::message_t zmqm((colorsizebyte + depthsizebyte + irsizebyte) * num_streams);
    socket.recv(&zmqm); // blocking





    std::vector<std::vector<unsigned char>> color_frames(num_streams, std::vector<unsigned char>(width_c * height_c * 3) );
    std::vector<std::vector<float>> depth_frames(num_streams, std::vector<float>(width_dir * height_dir) );

    unsigned offset = 0;
    for(unsigned cam_idx = 0; cam_idx < num_streams; ++cam_idx) {
      // color
      if(!rgb_is_compressed){
        memcpy(color_frames[cam_idx].data(), (zmqm.data() + offset), colorsizebyte);
        cv::imshow((std::string("color@") + std::to_string(cam_idx) ).c_str(), cv::Mat(height_c, width_c, CV_8UC3, convertRGB2BGR( (unsigned char *) (color_frames[cam_idx].data()), width_c, height_c)));
      }
      offset += colorsizebyte;
      
      memcpy(depth_frames[cam_idx].data(), (zmqm.data() + offset), depthsizebyte);
      cv::imshow((std::string("depth@") + std::to_string(cam_idx) ).c_str(), cv::Mat(height_dir, width_dir, CV_8UC1, convertTo8Bit( (float * ) (depth_frames[cam_idx].data()), width_dir, height_dir)));
      offset += depthsizebyte;
    }



    glPointSize(1.0);
    glBegin(GL_POINTS);

    int num_cameras_to_iterate = std::min(num_active_cameras, num_streams);

    glm::vec3 inv_view_axis = glm::vec3(0.0, 0.0, 1.0);

    for(unsigned s_num = 0; s_num < num_cameras_to_iterate; ++s_num){
      // do 3D recosntruction for each depth pixel
      for(unsigned y = 0; y < height_dir; ++y){
          for(unsigned x = 0; x < (width_dir - 3); ++x){
            const unsigned d_idx = y * width_dir + x;

            float depth_center = depth_frames[s_num][d_idx];

            if(depth_center > distance_cut_off_threshold) {
              continue;
            }

            unsigned int idx_left  = y * width_dir  +  x - 1;
            unsigned int idx_right = y * width_dir + x + 1;

            if(x == 0) {
              idx_left = y * width_dir  +  x;
            }
            if(x >= width_dir - 3) {
              idx_right = y * width_dir  +  x;
            }

            float depth_left  =  depth_frames[s_num][idx_left];
            float depth_right =  depth_frames[s_num][idx_right];

            depth_left = ( (depth_left > 0.0) && (depth_left < distance_cut_off_threshold) ) ? depth_left : depth_center;
            depth_right = ( (depth_right > 0.0) && (depth_right < distance_cut_off_threshold) ) ? depth_right : depth_center;

            unsigned int idx_bottom = (y-1) * width_dir + x;
            unsigned int idx_top    = (y+1) * width_dir + x;

            if(y == 0) {
              idx_bottom = y * width_dir  +  x;
            }
            if(y == height_dir) {
              idx_top = y * width_dir     +  x;
            }

            float depth_bottom  =  depth_frames[s_num][idx_bottom];
            float depth_top     =  depth_frames[s_num][idx_top];

            depth_bottom = ( (depth_bottom > 0.0) && (depth_bottom < distance_cut_off_threshold) ) ? depth_bottom : depth_center;
            depth_top    = ( (depth_top > 0.0) && (depth_top < distance_cut_off_threshold) ) ? depth_top : depth_center;


            auto image_to_3d = [&](float x, float y, float d) -> glm::vec3 { 

              const float x_d  = ((x - sensor_configs[s_num].principal_d.x)/sensor_configs[s_num].focal_d.x) * d;
              const float y_d  = ((y - sensor_configs[s_num].principal_d.y)/sensor_configs[s_num].focal_d.y) * d;
              return glm::vec3(x_d, y_d, d);
            };

            //current world pos
            glm::vec3 current_world_pos3D = image_to_3d(x, y, depth_center);

            glm::vec3 left_world_pos3D = image_to_3d(x-1, y, depth_left);
            glm::vec3 right_world_pos3D = image_to_3d(x+1, y, depth_right);
            glm::vec3 top_world_pos3D = image_to_3d(x, y+1, depth_top);
            glm::vec3 bottom_world_pos3D = image_to_3d(x, y-1, depth_bottom);

            glm::vec3 vertical_gradient = bottom_world_pos3D - top_world_pos3D;
            glm::vec3 horizontal_gradient = left_world_pos3D - right_world_pos3D;

            if(    glm::length(vertical_gradient) > gradient_cut_off_threshold 
                || glm::length(horizontal_gradient) > gradient_cut_off_threshold) {
              continue;
            }

            glm::vec3 normal = glm::normalize(glm::cross(vertical_gradient, 
                                                         horizontal_gradient));

            float gray_value = depth_center / 12.0;

            glm::vec3 gray_color(gray_value, gray_value, gray_value);
            //glColor3f(gray_value, gray_value, gray_value);

            glm::vec3 normal_color = glm::mix(gray_color, glm::vec3(0.5, 0.5, 0.5)*(glm::vec3(1.0, 1.0, 1.0) + normal), mix_in_factor);
            glColor3f(normal_color.x, normal_color.y, normal_color.z);

            glVertex3f(current_world_pos3D.x, current_world_pos3D.y, current_world_pos3D.z);

          }
        }
      }
    glEnd();

    win.update();
    cv::waitKey(1);
  }

  return 0;
}
