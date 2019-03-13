#include <CMDParser.hpp>
#include <zmq.hpp>

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

int main(int argc, char* argv[]){

  unsigned width_dir = 512;
  unsigned height_dir = 424;
  unsigned width_c = 1280;
  unsigned height_c = 1080;

  unsigned colorsizebyte = 0;
  unsigned depthsizebyte = 0;
  unsigned irsizebyte = 0;
  unsigned num_kinect_cameras = 1;
  bool rgb_is_compressed = false;
  CMDParser p("serverport");
  p.addOpt("k",1,"num_kinect_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compression for rgb stream, default: false");
  p.addOpt("r",4,"realsense", "enable display for realsense cameras and specify resolution of color and depth sensor e.g. 1280 720 1280 720, default: Kinect V2");
  p.addOpt("i",-1,"infrared", "enable infrared");
  p.init(argc,argv);

  if(p.isOptSet("k")){
    num_kinect_cameras = p.getOptsInt("k")[0];
  }
  if(p.isOptSet("c")){
    rgb_is_compressed = true;
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
  std::string endpoint("tcp://" + p.getArgs()[0]);
  socket.connect(endpoint.c_str());


  bool running = true;
  while(running){

    zmq::message_t zmqm((colorsizebyte + depthsizebyte + irsizebyte) * num_kinect_cameras);
    socket.recv(&zmqm); // blocking

    unsigned offset = 0;
    for(unsigned i = 0; i < num_kinect_cameras; ++i){
      // color
      if(!rgb_is_compressed){
        cv::imshow((std::string("color@") + std::to_string(i) ).c_str(), cv::Mat(height_c, width_c, CV_8UC3, convertRGB2BGR( (unsigned char *) (zmqm.data() + offset), width_c, height_c)));
      }
      offset += colorsizebyte;
      // depth
      cv::imshow((std::string("depth@") + std::to_string(i) ).c_str(), cv::Mat(height_dir, width_dir, CV_8UC1, convertTo8Bit( (float * ) (zmqm.data() + offset), width_dir, height_dir)));
      offset += depthsizebyte;
      // infrared
      if(irsizebyte){
        cv::imshow((std::string("ir@")  + std::to_string(i) ).c_str(), cv::Mat(height_dir, width_dir, CV_8UC1, (unsigned char *) (zmqm.data() + offset)));
        offset += irsizebyte;
      }

    }
    cv::waitKey(1);
  }

  return 0;
}
