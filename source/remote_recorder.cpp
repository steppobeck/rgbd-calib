#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <ChronoMeter.hpp>
#include <udpconnection.hpp>
#include <RemoteCommands.hpp>

#include <zmq.hpp>

#include <iostream>
#include <string.h>

#define MAX_FRAMES_TO_RECORD 1800


FileBuffer* fb = nullptr;
size_t framecounter = 0;
void close(){
  if(fb != nullptr){
    fb->close();  
    delete fb;
    fb = nullptr;
    std::cout << "closed" << std::endl;
  }
}

void re_open(const char* fn){
  close();
  fb = new FileBuffer(fn);
  if(!fb->open("w", 0)){
    std::cerr << "error opening " << fn << " exiting..." << std::endl;
    exit(0);
  }
  std::cout << "opened" << std::endl;
  framecounter = 0;
}



int main(int argc, char* argv[]){


  unsigned num_kinect_cameras = 1;
  bool rgb_is_compressed = false;
  unsigned avango_command_port = 8000;
  CMDParser p("record_to_this_filename serverport");
  p.addOpt("k",1,"num_kinect_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.addOpt("a",1,"avango_command_port", "specify the command networking port from which this instance is controlled over network, default: 8000");
  p.addOpt("c",-1,"rgb_is_compressed", "enable compressed recording for rgb stream, default: false");
  p.init(argc,argv);

  
  if(p.isOptSet("k")){
    num_kinect_cameras = p.getOptsInt("k")[0];
  }
  
  if(p.isOptSet("a")){
    avango_command_port = p.getOptsInt("a")[0];
  }

  if(p.isOptSet("c")){
    rgb_is_compressed = true;
  }

  const unsigned colorsize = rgb_is_compressed ? 691200 : 1280 * 1080 * 3;
  const unsigned depthsize = 512 * 424 * sizeof(float);
  const unsigned framesize = (colorsize + depthsize) * num_kinect_cameras;

  const std::string filename(p.getArgs()[0]);




  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_SUB); // means a subscriber
  socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_RCVHWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + p.getArgs()[1]);
  socket.connect(endpoint.c_str());

  udpconnection command_recv;
  bool ret = command_recv.open_receiving_socket(avango_command_port);
  if(!ret){
        std::cerr << "ERROR could not open_receiving_socket on port " << avango_command_port << " exiting!" << std::endl;
        return 0;
  }
  

  ChronoMeter cm;

  bool running = true;
  int last_command = RemoteCommands::STOP;
  while(running){

    zmq::message_t zmqm(framesize);
    socket.recv(&zmqm); // blocking
    int current_command = RemoteCommands::STOP;
    size_t bytes_received = command_recv.recv(&current_command, sizeof(current_command), false /*do no block*/);
    if(bytes_received == sizeof(current_command)){
      if(current_command != last_command){
        last_command = current_command;
        switch(last_command){
          case RemoteCommands::STOP:
            std::cout << "stop record" << std::endl;
            close();
            break;
          case RemoteCommands::RECORD:
            std::cout << "start record" << std::endl;
            re_open(filename.c_str());
            break;
          default:
            break;
        }
      }
    }

    switch(last_command){
      case RemoteCommands::RECORD:
        {
          const double currtime = cm.getTick();
          memcpy((char*) zmqm.data(), (const char*) &currtime, sizeof(double));
          if(fb == nullptr){
            std::cerr << "ERROR: not recording, file not open!" << std::endl;
          }
          else{
            //std::cout << "recording frame " << ++framecounter << " at time " << currtime << std::endl;
            fb->write((unsigned char*) zmqm.data(), framesize);
            if(framecounter > MAX_FRAMES_TO_RECORD){
              last_command = RemoteCommands::STOP;
              close();
              std::cerr << "ALTERT: closed recording since too much frames were already recorded!: " << framecounter << std::endl;
            }
          }
          
        }
        break;
      default:
        break;
    }



  }

  close();

  return 0;
}
