#include <CMDParser.hpp>
#include <FileBuffer.hpp>

#include <timevalue.hpp>
#include <clock.hpp>
#include <zmq.hpp>

#include <iostream>

int main(int argc, char* argv[]){
  unsigned num_kinect_cameras = 1;
  bool rgb_is_compressed = false;
  float max_fps = 20.0;
  CMDParser p("play_this_filename serverport");
  p.addOpt("k",1,"num_kinect_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.addOpt("f",1,"max_fps", "specify how fast in fps the stream should be played, default: 20.0");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed recording for rgb stream, default: false");
  p.init(argc,argv);

  if(p.isOptSet("k")){
    num_kinect_cameras = p.getOptsInt("k")[0];
  }
  if(p.isOptSet("f")){
    max_fps = p.getOptsFloat("f")[0];
  }
  if(p.isOptSet("c")){
    rgb_is_compressed = true;
  }

  unsigned min_frame_time_ns = 1000000000/max_fps;

  const unsigned colorsize = rgb_is_compressed ? 691200 : 1280 * 1080 * 3;
  const unsigned depthsize = 512 * 424 * sizeof(float);

  FileBuffer fb(p.getArgs()[0].c_str());
  if(!fb.open("r")){
    std::cerr << "error opening " << p.getArgs()[0] << " exiting..." << std::endl;
    return 1;
  }
  fb.setLooping(true);


  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a subscriber
  uint64_t hwm = 1; 
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + p.getArgs()[1]);
  socket.bind(endpoint.c_str());

  sensor::timevalue ts(sensor::clock::time());


  while(true){



    zmq::message_t zmqm((colorsize + depthsize) * num_kinect_cameras);
    
    unsigned offset = 0;
    for(unsigned i = 0; i < num_kinect_cameras; ++i){
      fb.read((unsigned char*) zmqm.data() + offset, colorsize);
      offset += colorsize;
      fb.read((unsigned char*) zmqm.data() + offset, depthsize);
      offset += depthsize;
    }

    // send frames
    socket.send(zmqm);
    
    // check if fps is correct
    sensor::timevalue ts_now = sensor::clock::time();
    long long time_spent_ns = (ts_now - ts).nsec();
    long long rest_sleep_ns = min_frame_time_ns - time_spent_ns;
    ts = ts_now;
    if(0 < rest_sleep_ns){
      sensor::timevalue rest_sleep(0,rest_sleep_ns);
      nanosleep(rest_sleep);
    }

  }

  return 0;
}
