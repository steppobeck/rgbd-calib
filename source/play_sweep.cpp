#include <CMDParser.hpp>
#include <FileBuffer.hpp>

#include <timevalue.hpp>
#include <clock.hpp>
#include <zmq.hpp>

#include <iostream>

int main(int argc, char* argv[]){
  float max_fps = 20.0;
  CMDParser p("play_this_filename serverport");
  p.addOpt("f",1,"max_fps", "specify how fast in fps the stream should be played, default: 20.0");
  p.init(argc,argv);

  if(p.isOptSet("f")){
    max_fps = p.getOptsFloat("f")[0];
  }

  unsigned min_frame_time_ns = 1000000000/max_fps;

  const unsigned colorsize = 1280 * 1080 * 3;
  const unsigned depthsize = 512 * 424 * sizeof(float);
  const unsigned irsize    = 512 * 424;
  unsigned char* irframe   = new unsigned char [irsize];
  double time_stamp = 0.0;

  FileBuffer fb(p.getArgs()[0].c_str());
  if(!fb.open("r")){
    std::cerr << "error opening " << p.getArgs()[0] << " exiting..." << std::endl;
    return 1;
  }
  fb.setLooping(true);


  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher
#if ZMQ_VERSION_MAJOR < 3
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
#else
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
#endif 
  std::string endpoint("tcp://" + p.getArgs()[1]);
  socket.bind(endpoint.c_str());

  sensor::timevalue ts(sensor::clock::time());


  while(true){



    zmq::message_t zmqm(colorsize + depthsize);
    
    fb.read(&time_stamp, sizeof(double));
    fb.read((unsigned char*) zmqm.data() , colorsize);
    fb.read(&time_stamp, sizeof(double));
    fb.read((unsigned char*) zmqm.data() + colorsize, depthsize);
    fb.read(irframe, irsize);
    

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
