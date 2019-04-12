#include <CMDParser.hpp>
#include <zmq.hpp>

#include <rgbdsensor.hpp>
#include <window.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <string.h>

std::vector<std::pair<float, glm::mat4> > sensor_world_extrinsics;

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

glm::mat4
to_glm(const Eigen::Matrix4f& t){
  glm::mat4 result;
  // assuming column major matrices in both types
  for(unsigned c = 0; c < 4; ++c){
    for(unsigned r = 0; r < 4; ++r){
      result[c][r] = t(r,c);
    }
  }
  return result;
}


std::pair<float, glm::mat4>
icp_step(const std::vector<glm::vec3>& pcA, const std::vector<glm::vec3>& pcB, unsigned max_itertions){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ>);


  // fill cloudA
  cloudA->width    = pcA.size();
  cloudA->height   = 1;
  cloudA->is_dense = false; // Specifies if all the data in points is finite (true), or whether the XYZ values of certain points might contain Inf/NaN values (false).
  cloudA->points.resize (cloudA->width * cloudA->height);
  for (size_t i = 0; i < pcA.size(); ++i)
  {
    const glm::vec3& p = pcA[i];
    cloudA->points[i].x = p[0];
    cloudA->points[i].y = p[1];
    cloudA->points[i].z = p[2];
  }


  // fill cloudB
  cloudB->width    = pcB.size();
  cloudB->height   = 1;
  cloudB->is_dense = false;
  cloudB->points.resize (cloudB->width * cloudB->height);
  for (size_t i = 0; i < pcB.size(); ++i)
  {
    const glm::vec3& p = pcB[i];
    cloudB->points[i].x = p[0];
    cloudB->points[i].y = p[1];
    cloudB->points[i].z = p[2];
  }

  std::cout << "performing icp for cloudA(" << pcA.size() << ") to cloudB(" << pcB.size() << ")" << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloudA);
  icp.setInputTarget(cloudB);
  icp.setMaximumIterations(max_itertions);
  icp.setRANSACOutlierRejectionThreshold(0.1);
  icp.setMaxCorrespondenceDistance(100 * 0.1);
  // icp.setRANSACOutlierRejectionThreshold(double inlier_threshold); The method considers a point to be an inlier, if the distance between the target data index and the transformed source index is smaller than the given inlier distance threshold. The value is set by default to 0.05m.   
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::pair<float, glm::mat4> result;
  result.first = std::numeric_limits<float>::max();
  if(!icp.hasConverged()){
    return result;
  }

  result.first = icp.getFitnessScore();
  result.second = to_glm(icp.getFinalTransformation());
  std::cout << "icp score: " << result.first << std::endl;
  std::cout << "a -> b: " << result.second << std::endl;
  return result;
}


bool world_extrinsics_computing = false;
void
register_a_to_b(unsigned extrinsic_num, std::vector<glm::vec3> a, std::vector<glm::vec3> b){
  std::pair<float, glm::mat4> r = icp_step(a, b, 50);
  if(r.first < sensor_world_extrinsics[extrinsic_num].first){
    sensor_world_extrinsics[extrinsic_num] = r;
    std::cout << "updated extrinsic "  << sensor_world_extrinsics[extrinsic_num].first << std::endl;  
  }
  world_extrinsics_computing = false;
}


void
start_calibration(const std::vector<std::vector<glm::vec3> >& point_clouds){
  static boost::thread* t = nullptr;
  if(t != nullptr){
    t->join();
    delete t;
    t = nullptr;
  }
  for(unsigned i = 1; i < point_clouds.size(); ++i){
    t = new boost::thread(boost::bind(&register_a_to_b, i, point_clouds[i], point_clouds[0]));
  }
}


}





int main(int argc, char* argv[]){



  CMDParser p("calibfile.cv_yml ...");
  p.addOpt("s",1,"serverport", "Serverport to receive stream from");
  p.addOpt("d",2,"display_size", "Size of the window, default: 800 800");
  p.addOpt("c",1,"cut_off_threshold", "Distance Cut-Off Threshold in meters, default 10.0");
  p.addOpt("g",1,"gradient_cut_off_threshold", "Gradient cut off threshold, default: FLT_MAX");

  p.init(argc,argv);

  unsigned num_streams = p.getArgs().size();
  std::vector<RGBDConfig> sensor_configs(num_streams);
  sensor_world_extrinsics.resize(num_streams);
  std::vector<std::vector<glm::vec3> > point_clouds(num_streams);
  for(int sensor_idx = 0; sensor_idx < num_streams; ++sensor_idx) {
    sensor_configs[sensor_idx].read(p.getArgs()[sensor_idx].c_str());

    sensor_world_extrinsics[sensor_idx].first = std::numeric_limits<float>::max();
    sensor_world_extrinsics[sensor_idx].second = glm::mat4();
  }
  


  float distance_cut_off_threshold = 10.0;
  if(p.isOptSet("c")){
    distance_cut_off_threshold = (p.getOptsFloat("c")[0]);
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

  
  unsigned colorsizebyte = sensor_configs[0].size_rgb.x * sensor_configs[0].size_rgb.y * 3;
  unsigned depthsizebyte = sensor_configs[0].size_d.x * sensor_configs[0].size_d.y * sizeof(float);


  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM,&hwm, sizeof(hwm));
  std::string endpoint(std::string("tcp://") +  p.getOptsString("s")[0] );
  socket.connect(endpoint.c_str());



  Window win(window_size, true /*3D mode*/);

  while (!win.shouldClose()) {

    auto t = win.getTime();
    if (win.isKeyPressed(GLFW_KEY_ESCAPE)) {
      win.stop();
    }

    zmq::message_t zmqm((colorsizebyte + depthsizebyte) * num_streams);
    socket.recv(&zmqm); // blocking





    std::vector<std::vector<unsigned char>> color_frames(num_streams, std::vector<unsigned char>(sensor_configs[0].size_rgb.x * sensor_configs[0].size_rgb.y * 3) );
    std::vector<std::vector<float>> depth_frames(num_streams, std::vector<float>(sensor_configs[0].size_d.x * sensor_configs[0].size_d.y) );

    unsigned offset = 0;
    for(unsigned cam_idx = 0; cam_idx < num_streams; ++cam_idx) {
      // color

      memcpy(color_frames[cam_idx].data(), (zmqm.data() + offset), colorsizebyte);
      cv::imshow((std::string("color@") + std::to_string(cam_idx) ).c_str(), cv::Mat(sensor_configs[0].size_rgb.y, sensor_configs[0].size_rgb.x, CV_8UC3, convertRGB2BGR( (unsigned char *) (color_frames[cam_idx].data()), sensor_configs[0].size_rgb.x, sensor_configs[0].size_rgb.y)));

      offset += colorsizebyte;
      
      memcpy(depth_frames[cam_idx].data(), (zmqm.data() + offset), depthsizebyte);
      cv::imshow((std::string("depth@") + std::to_string(cam_idx) ).c_str(), cv::Mat(sensor_configs[0].size_d.y, sensor_configs[0].size_d.x, CV_8UC1, convertTo8Bit( (float * ) (depth_frames[cam_idx].data()), sensor_configs[0].size_d.x, sensor_configs[0].size_d.y)));
      offset += depthsizebyte;

    }



    



    glPointSize(1.0);
    glBegin(GL_POINTS);

    

    for(unsigned s_num = 0; s_num < num_streams; ++s_num){

      // do 3D recosntruction for each depth pixel
      point_clouds[s_num].clear();
      if(1 == s_num){
        static float last_score = sensor_world_extrinsics[s_num].first;
        if(last_score > sensor_world_extrinsics[s_num].first){
          std::cout << "extrinsic changed from score " << last_score << " to " << sensor_world_extrinsics[s_num].first << std::endl;
          std::cout << sensor_world_extrinsics[s_num].second << std::endl;
          last_score = sensor_world_extrinsics[s_num].first;
        }
      }
      for(unsigned y = 0; y < sensor_configs[s_num].size_d.y; ++y){
          for(unsigned x = 0; x < (sensor_configs[s_num].size_d.x - 3); ++x){

            const unsigned d_idx = y * sensor_configs[s_num].size_d.x + x;

            float depth_center = depth_frames[s_num][d_idx];

            if(depth_center > distance_cut_off_threshold) {
              continue;
            }

            auto image_to_3d = [&](float x, float y, float d) -> glm::vec3 { 

              const float x_d  = ((x - sensor_configs[s_num].principal_d.x)/sensor_configs[s_num].focal_d.x) * d;
              const float y_d  = ((y - sensor_configs[s_num].principal_d.y)/sensor_configs[s_num].focal_d.y) * d;
              return glm::vec3(x_d, y_d, d);
            };

            //current world pos
            glm::vec3 current_world_pos3D = image_to_3d(x, y, depth_center);


            // filter at depth silhouttes aka flying pixels based on gradient
#if 1
            unsigned int idx_left  = y * sensor_configs[s_num].size_d.x  +  x - 1;
            unsigned int idx_right = y * sensor_configs[s_num].size_d.x + x + 1;

            if(x == 0) {
              idx_left = y * sensor_configs[s_num].size_d.x  +  x;
            }
            if(x >= sensor_configs[s_num].size_d.x - 3) {
              idx_right = y * sensor_configs[s_num].size_d.x  +  x;
            }

            float depth_left  =  depth_frames[s_num][idx_left];
            float depth_right =  depth_frames[s_num][idx_right];

            depth_left = ( (depth_left > 0.0) && (depth_left < distance_cut_off_threshold) ) ? depth_left : depth_center;
            depth_right = ( (depth_right > 0.0) && (depth_right < distance_cut_off_threshold) ) ? depth_right : depth_center;

            unsigned int idx_bottom = (y-1) * sensor_configs[s_num].size_d.x + x;
            unsigned int idx_top    = (y+1) * sensor_configs[s_num].size_d.x + x;

            if(y == 0) {
              idx_bottom = y * sensor_configs[s_num].size_d.x  +  x;
            }
            if(y == sensor_configs[s_num].size_d.y) {
              idx_top = y * sensor_configs[s_num].size_d.x     +  x;
            }

            float depth_bottom  =  depth_frames[s_num][idx_bottom];
            float depth_top     =  depth_frames[s_num][idx_top];

            depth_bottom = ( (depth_bottom > 0.0) && (depth_bottom < distance_cut_off_threshold) ) ? depth_bottom : depth_center;
            depth_top    = ( (depth_top > 0.0) && (depth_top < distance_cut_off_threshold) ) ? depth_top : depth_center;

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
#endif

            // calculate normal
#if 0
            glm::vec3 normal = glm::normalize(glm::cross(vertical_gradient, 
                                                         horizontal_gradient));
            float gray_value = depth_center / 12.0;
            glm::vec3 gray_color(gray_value, gray_value, gray_value);
            glm::vec3 normal_color = glm::mix(gray_color, glm::vec3(0.5, 0.5, 0.5)*(glm::vec3(1.0, 1.0, 1.0) + normal), mix_in_factor);
#endif

            // calculate color coordinate
            glm::vec4 pos_d_H(current_world_pos3D.x, current_world_pos3D.y, current_world_pos3D.z, 1.0f);
            glm::vec4 pos_rgb_H = sensor_configs[s_num].eye_d_to_eye_rgb * pos_d_H;    
            float xcf = (pos_rgb_H[0]/pos_rgb_H[2]) * sensor_configs[s_num].focal_rgb.x + sensor_configs[s_num].principal_rgb.x;
            float ycf = (pos_rgb_H[1]/pos_rgb_H[2]) * sensor_configs[s_num].focal_rgb.y + sensor_configs[s_num].principal_rgb.y;
            
            // filter based on color field of view
            if(xcf < 0.0 || xcf > (sensor_configs[s_num].size_rgb.x - 1.0f) ||
               ycf < 0.0 || ycf > (sensor_configs[s_num].size_rgb.y - 1.0f)){
              continue;
            }
            else{
              xcf = std::max(0.0f, std::min(xcf, sensor_configs[s_num].size_rgb.x - 1.0f));
              ycf = std::max(0.0f, std::min(ycf, sensor_configs[s_num].size_rgb.y - 1.0f));  
            }
            glm::vec2 pos_rgb(xcf,ycf);

            // lookup color
            glm::vec3 rgb;
            {

              // calculate weights and boundaries along x direction
              const unsigned xa = std::floor(pos_rgb.x);
              const unsigned xb = std::ceil(pos_rgb.x);
              const float w_xb = (pos_rgb.x - xa);
              const float w_xa = (1.0 - w_xb);

              // calculate weights and boundaries along y direction
              const unsigned ya = std::floor(pos_rgb.y);
              const unsigned yb = std::ceil(pos_rgb.y);
              const float w_yb = (pos_rgb.y - ya);
              const float w_ya = (1.0 - w_yb);


              unsigned char* frame_rgb_real = color_frames[s_num].data();

              // calculate indices to access data
              const unsigned idmax = 3u * sensor_configs[s_num].size_rgb.x * sensor_configs[s_num].size_rgb.y - 2u;
              const unsigned id00 = std::min( ya * 3u * sensor_configs[s_num].size_rgb.x + 3u * xa  , idmax);
              const unsigned id10 = std::min( ya * 3u * sensor_configs[s_num].size_rgb.x + 3u * xb  , idmax);
              const unsigned id01 = std::min( yb * 3u * sensor_configs[s_num].size_rgb.x + 3u * xa  , idmax);
              const unsigned id11 = std::min( yb * 3u * sensor_configs[s_num].size_rgb.x + 3u * xb  , idmax);
              

              // RED CHANNEL
              {
                // 1. interpolate between x direction;
                const float tmp_ya = w_xa * frame_rgb_real[id00] + w_xb * frame_rgb_real[id10];
                const float tmp_yb = w_xa * frame_rgb_real[id01] + w_xb * frame_rgb_real[id11];
                // 2. interpolate between y direction;
                rgb.x = w_ya * tmp_ya + w_yb * tmp_yb;
              }

              // GREEN CHANNEL
              {
                // 1. interpolate between x direction;
                const float tmp_ya = w_xa * frame_rgb_real[id00 + 1] + w_xb * frame_rgb_real[id10 + 1];
                const float tmp_yb = w_xa * frame_rgb_real[id01 + 1] + w_xb * frame_rgb_real[id11 + 1];
                // 2. interpolate between y direction;
                rgb.y = w_ya * tmp_ya + w_yb * tmp_yb;
              }

              // BLUE CHANNEL
              {
                // 1. interpolate between x direction;
                const float tmp_ya = w_xa * frame_rgb_real[id00 + 2] + w_xb * frame_rgb_real[id10 + 2];
                const float tmp_yb = w_xa * frame_rgb_real[id01 + 2] + w_xb * frame_rgb_real[id11 + 2];
                // 2. interpolate between y direction;
                rgb.z = w_ya * tmp_ya + w_yb * tmp_yb;
              }

              rgb.x /= 255.0;
              rgb.y /= 255.0;
              rgb.z /= 255.0;

            }

            glColor3f(rgb.x, rgb.y, rgb.z);

            point_clouds[s_num].push_back(current_world_pos3D);
            glm::vec4 calibrated_world_pos = sensor_world_extrinsics[s_num].second * glm::vec4(current_world_pos3D.x, current_world_pos3D.y, current_world_pos3D.z, 1.0);

            glVertex3f(calibrated_world_pos.x, calibrated_world_pos.y, calibrated_world_pos.z);

          }
        }
      }
    glEnd();


    static bool calibrate = false;
    if(win.isKeyPressed(67 /*"c"*/)){
      calibrate = true;
    }
    if(!world_extrinsics_computing && calibrate){
      start_calibration(point_clouds);
      world_extrinsics_computing = true;
    }

    win.update();
    cv::waitKey(1);
  }

  return 0;
}
