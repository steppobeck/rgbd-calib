#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <ChronoMeter.hpp>
#include <zmq.hpp>

#include <iostream>
#include <string.h>

int main(int argc, char* argv[]){

  unsigned wait_frames_to_before_start = 0;
  unsigned num_kinect_cameras = 1;
  unsigned num_seconds_to_record = 10;
  bool rgb_is_compressed = false;
  CMDParser p("record_to_this_filename serverport");
  p.addOpt("k",1,"num_kinect_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.addOpt("n",1,"num_seconds_to_record", "specify how many seconds should be recorded, default: 10");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed recording for rgb stream, default: false");
  p.addOpt("w",1,"wait_frames_to_before_start", "specify how many seconds to wait before start, default: 0");
  p.init(argc,argv);

  if(p.isOptSet("w")){
    wait_frames_to_before_start = p.getOptsInt("w")[0];
  }
  if(p.isOptSet("k")){
    num_kinect_cameras = p.getOptsInt("k")[0];
  }
  if(p.isOptSet("n")){
    num_seconds_to_record = p.getOptsInt("n")[0];
  }
  if(p.isOptSet("c")){
    rgb_is_compressed = true;
  }

  const unsigned colorsize = rgb_is_compressed ? 691200 : 1280 * 1080 * 3;
  const unsigned depthsize = 512 * 424 * sizeof(float);

  FileBuffer fb(p.getArgs()[0].c_str());
  if(!fb.open("w", 0/*1073741824 1 GB buffer*/)){
    std::cerr << "error opening " << p.getArgs()[0] << " exiting..." << std::endl;
    return 1;
  }


  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + p.getArgs()[1]);
  socket.connect(endpoint.c_str());

  while(wait_frames_to_before_start > 0){

    zmq::message_t zmqm((colorsize + depthsize) * num_kinect_cameras);
    socket.recv(&zmqm); // blocking

    std::cout << "countdown: " << wait_frames_to_before_start << std::endl;

    --wait_frames_to_before_start;
  }

  std::cout << "START!!!!!!!!!!!!!!!!!!!!!!!!!!!!: " << std::endl;

  ChronoMeter cm;
  const double starttime = cm.getTick();
  bool running = true;
  while(running){

    zmq::message_t zmqm((colorsize + depthsize) * num_kinect_cameras);
    socket.recv(&zmqm); // blocking

    const double currtime = cm.getTick();
    const double elapsed = currtime - starttime;
    std::cout << "remaining seconds: " << num_seconds_to_record - elapsed << std::endl;
    if(elapsed > num_seconds_to_record)
      running = false;
    memcpy((char*) zmqm.data(), (const char*) &currtime, sizeof(double));

    unsigned offset = 0;
    for(unsigned i = 0; i < num_kinect_cameras; ++i){
      fb.write((unsigned char*) zmqm.data() + offset, colorsize);
      offset += colorsize;
      fb.write((unsigned char*) zmqm.data() + offset, depthsize);
      offset += depthsize;
    }



  }

  fb.close();

  return 0;
}
