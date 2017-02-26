#include <CMDParser.hpp>
#include <FileBuffer.hpp>

#include <timevalue.hpp>
#include <clock.hpp>
#include <zmq.hpp>

#include <iostream>
#include <sstream>

namespace{

  template <class T>
  inline std::string
  toString(T value){
    std::ostringstream stream;
    stream << value;
    return stream.str();
  }

}

int main(int argc, char* argv[]){
  int start_loop = 0;
  int end_loop = 0;
  int num_loops = 0;
  bool perform_loop = false;
  bool swing = false;
  unsigned num_kinect_cameras = 1;
  bool rgb_is_compressed = false;
  float max_fps = 20.0;
  std::string socket_ip = "127.0.0.01";
  unsigned base_socket_port = 7000;
  CMDParser p("play_this_filename ...");
  p.addOpt("k",1,"num_kinect_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.addOpt("f",1,"max_fps", "specify how fast in fps the stream should be played, default: 20.0");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed recording for rgb stream, default: false");

  p.addOpt("s",1,"socket_ip", "specify ip address of socket for sending, default: " + socket_ip);
  p.addOpt("p",1,"socket_port", "specify port of socket for sending, default: " + toString(base_socket_port));
  p.addOpt("l",2,"loop", "specify a start and end frame for looping, default: " + toString(start_loop) + " " + toString(end_loop));
  p.addOpt("w",-1,"swing", "enable swing looping mode, default: false");
  p.addOpt("n",1,"num_loops", "loop n time, default: loop forever");
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

  if(p.isOptSet("s")){
    socket_ip = p.getOptsString("s")[0];
  }

  if(p.isOptSet("p")){
    base_socket_port = p.getOptsInt("p")[0];
  }

  if(p.isOptSet("l")){
    start_loop = p.getOptsInt("l")[0];
    end_loop = p.getOptsInt("l")[1];
    if(start_loop > end_loop || start_loop < 0 || end_loop < 0){
      std::cerr << "ERROR: -l option must be both positive and second parameter must be greater than first!" << std::endl;
      p.showHelp();
    }
    perform_loop = true;
  }

  if(p.isOptSet("w")){
    swing = true;
  }

  if(p.isOptSet("n")){
    num_loops = p.getOptsInt("n")[0];
  }

  unsigned min_frame_time_ns = 1000000000/max_fps;

  const unsigned colorsize = rgb_is_compressed ? 691200 : 1280 * 1080 * 3;
  const unsigned depthsize = 512 * 424 * sizeof(float);
  const size_t frame_size_bytes((colorsize + depthsize) * num_kinect_cameras);

  zmq::context_t ctx(1); // means single threaded

  const unsigned num_streams = p.getArgs().size();

  std::cout << "going to stream " << num_streams << " to socket ip " << socket_ip << " starting at port number " << base_socket_port << std::endl;

  std::vector<FileBuffer*> fbs;
  std::vector<zmq::socket_t* > sockets;
  std::vector<int> frame_numbers;
  for(unsigned s_num = 0; s_num < num_streams; ++s_num){
    FileBuffer* fb = new FileBuffer(p.getArgs()[s_num].c_str());
    if(!fb->open("r")){
      std::cerr << "error opening " << p.getArgs()[s_num] << " exiting..." << std::endl;
      return 1;
    }
    else{
      const unsigned n_fs = fb->getFileSizeBytes()/frame_size_bytes;
      std::cout << p.getArgs()[s_num] << " contains " << n_fs << " frames..."  << std::endl;
      if(perform_loop && end_loop > 0 && end_loop > n_fs){
	end_loop = n_fs;
	start_loop = std::min(end_loop, start_loop);
	std::cout << "INFO: setting start loop to " << start_loop << " and end loop to " << end_loop << std::endl;
      }
    }
    fb->setLooping(true);
    fbs.push_back(fb);
    frame_numbers.push_back(0);
    zmq::socket_t* socket = new zmq::socket_t(ctx, ZMQ_PUB); // means a publisher
    uint32_t hwm = 1;
    socket->setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
    std::string endpoint("tcp://" + socket_ip + ":" + toString(base_socket_port + s_num));
    socket->bind(endpoint.c_str());
    std::cout << "binding socket to " << endpoint << std::endl;
    sockets.push_back(socket);
  }

  
  bool fwd = true;
  int loop_num = 1;
  sensor::timevalue ts(sensor::clock::time());
  while(true){

    if(loop_num > num_loops && num_loops > 0){
      break;
    }
    
    for(unsigned s_num = 0; s_num < num_streams; ++s_num){

      if(perform_loop){
	frame_numbers[s_num] = fwd ? frame_numbers[s_num] + 1 : frame_numbers[s_num] - 1;
	if(frame_numbers[s_num] < start_loop){
	  frame_numbers[s_num] = start_loop + 1;
	  if(swing){
	    fwd = true;
	  }

	  if(s_num == 0){
	    ++loop_num;
	  }

	}
	else if(frame_numbers[s_num] > end_loop){
	  if(swing){
	    fwd = false;
	    frame_numbers[s_num] = end_loop - 1;
	  }
	  else{
	    frame_numbers[s_num] = start_loop;
	    if(s_num == 0){
	      ++loop_num;
	    }
	  }
	}
	std::cout << "s_num: " << s_num << " -> frame_number: " << frame_numbers[s_num] << std::endl;
	fbs[s_num]->gotoByte(frame_size_bytes * frame_numbers[s_num]);
      }

      zmq::message_t zmqm(frame_size_bytes);
      fbs[s_num]->read((unsigned char*) zmqm.data(), frame_size_bytes);
      // send frames
      sockets[s_num]->send(zmqm);
    }

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

  // cleanup
  for(unsigned s_num = 0; s_num < num_streams; ++s_num){
    delete fbs[s_num];
    delete sockets[s_num];
  }

  return 0;
}
