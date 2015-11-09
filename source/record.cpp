#include <CMDParser.hpp>
#include <FileBuffer.hpp>

#include <zmq.hpp>

#include <iostream>

int main(int argc, char* argv[]){
  unsigned num_kinect_cameras = 1;
  unsigned num_frames_to_record = 100;
  bool rgb_is_compressed = false;
  CMDParser p("record_to_this_filename serverport");
  p.addOpt("k",1,"num_kinect_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.addOpt("n",1,"num_frames_to_record", "specify how many frames should be recorded, default: 100");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed recording for rgb stream, default: false");
  p.init(argc,argv);

  if(p.isOptSet("k")){
    num_kinect_cameras = p.getOptsInt("k")[0];
  }
  if(p.isOptSet("n")){
    num_frames_to_record = p.getOptsInt("n")[0];
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

  while(num_frames_to_record > 0){
    std::cout << "remaining frames " << num_frames_to_record << std::endl;

    zmq::message_t zmqm((colorsize + depthsize) * num_kinect_cameras);
    socket.recv(&zmqm); // blocking

    unsigned offset = 0;
    for(unsigned i = 0; i < num_kinect_cameras; ++i){
      fb.write((unsigned char*) zmqm.data() + offset, colorsize);
      offset += colorsize;
      fb.write((unsigned char*) zmqm.data() + offset, depthsize);
      offset += depthsize;
    }
    --num_frames_to_record;
  }

  return 0;
}
