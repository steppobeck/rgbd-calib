#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <ChronoMeter.hpp>
#include <udpconnection.hpp>
#include <RemoteCommands.hpp>

#include <zmq.hpp>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <string.h>



ChronoMeter cm;

bool running = true;
int last_command = RemoteCommands::STOP;
int last_state = RemoteCommands::STOP;
double last_frame_time = cm.getTick();
double frametime;

FileBuffer* fb = nullptr;
size_t framecounter = 0;
size_t num_frames = 0;
void close(){
  if(fb != nullptr){
    fb->close();  
    delete fb;
    fb = nullptr;
    std::cout << "closed" << std::endl;
  }
}

void re_open(const char* fn, bool loop){
  close();
  fb = new FileBuffer(fn);
  if(!fb->open("r", 0)){
    std::cerr << "error opening " << fn << " exiting..." << std::endl;
    exit(0);
  }
  fb->setLooping(loop);
  std::cout << "opened" << std::endl;
  framecounter = 0;
}


void check_frametime(){
  const double elapsed_frame_time = frametime - last_frame_time;
  last_frame_time = frametime;
  const unsigned sleep_time = std::min(100u, std::max(0u, (unsigned)((elapsed_frame_time) * 1000u)));
  if(framecounter > 1){
      //std::cout << "sleep_time " << sleep_time << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
  } 
}


int main(int argc, char* argv[]){


  unsigned num_kinect_cameras = 1;
  bool rgb_is_compressed = false;
  unsigned avango_command_port = 8000;
  CMDParser p("play_this_filename serverport");
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
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a subscriber
  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + p.getArgs()[1]);
  socket.bind(endpoint.c_str());

  udpconnection command_recv;
  bool ret = command_recv.open_receiving_socket(avango_command_port);
  if(!ret){
        std::cerr << "ERROR could not open_receiving_socket on port " << avango_command_port << " exiting!" << std::endl;
        return 0;
  }
  


  while(running){


    int current_command = RemoteCommands::STOP;
    size_t bytes_received = command_recv.recv(&current_command, sizeof(current_command), false /*do no block*/);
    if(bytes_received == sizeof(current_command)){
      std::cout << "received: " << current_command << std::endl;
      if(current_command != last_command){
        last_command = current_command;
        switch(last_command){
          case RemoteCommands::STOP:
            close();
            last_state = RemoteCommands::STOP;
            std::cout << "stop" << std::endl;
            break;
          case RemoteCommands::PLAY:
            re_open(filename.c_str(), false);
            num_frames = fb->getFileSizeBytes()/framesize;
            last_state = RemoteCommands::PLAY;
            std::cout << "start play" << std::endl;
            break;
          case RemoteCommands::LOOP:
            re_open(filename.c_str(), true);
            num_frames = fb->getFileSizeBytes()/framesize;
            last_state = RemoteCommands::LOOP;
            std::cout << "start loop" << std::endl;
            break;
          case RemoteCommands::PAUSE:
            std::cout << "pause" << std::endl;
            break;
          case RemoteCommands::UNPAUSE:
            if(last_state != RemoteCommands::STOP){
              last_command = last_state;
              std::cout << "unpause" << std::endl;
            }
            break;
          default:
            break;
        }
      }
    }

    switch(last_command){
      case RemoteCommands::PLAY:
        {
          
          if(fb == nullptr){
            std::cerr << "ERROR: not recording, file not open!" << std::endl;
          }
          else{
            zmq::message_t zmqm(framesize);
            fb->read((unsigned char*) zmqm.data(), framesize);
            memcpy((char*) &frametime, (const char*) zmqm.data(), sizeof(double));
            check_frametime();
            //std::cout << "sending frame " << framecounter << " at time " << frametime << std::endl;
            socket.send(zmqm);
            ++framecounter;
            if(framecounter >= num_frames){
              std::cout << "stop play since end reached" << std::endl;
              last_command = RemoteCommands::STOP;
              last_state = RemoteCommands::STOP;
              close();
            }
          }
        }
        break;
      case RemoteCommands::LOOP:
        {
          //const double currtime = cm.getTick();
          if(fb == nullptr){
            std::cerr << "ERROR: not recording, file not open!" << std::endl;
          }
          else{
            zmq::message_t zmqm(framesize);
            fb->read((unsigned char*) zmqm.data(), framesize);
            memcpy((char*) &frametime, (const char*) zmqm.data(), sizeof(double));
            check_frametime();
            //std::cout << "looping frame " << framecounter << " at time " << frametime << std::endl;
            socket.send(zmqm);
            ++framecounter;
            if(framecounter >= num_frames){
              framecounter = 0;
            }
          }
        }
        break;
      default:
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        break;
    }

  }

  close();

  return 0;
}
