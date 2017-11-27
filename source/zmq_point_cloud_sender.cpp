#include <CMDParser.hpp>
#include <DataTypes.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <zmq.hpp>

#include <iostream>

const float one_dimensional_grid_resolution = 256;


const float delta_per_block = 1.0 / one_dimensional_grid_resolution;
const float two_times_delta_per_block = 2*delta_per_block;

// layout of points to send
struct XYZ32F_RGBA8 {
  float xyz[3];
  uint32_t rgba;
};

std::vector<XYZ32F_RGBA8> generate_points_on_implicit_sphere(float radius, size_t tick) {

  std::vector<XYZ32F_RGBA8> out_colored_points;

  uint64_t total_num_points = one_dimensional_grid_resolution * one_dimensional_grid_resolution * one_dimensional_grid_resolution;

  // reserve a maximal number of points, such that push back does not cause resizing all the time
  out_colored_points.reserve(total_num_points);


  // iterate discretely over 3D volume, step size = 1.0/one_dimensional_grid_resolution
  for(float z = -(radius + delta_per_block); z <= (radius + delta_per_block); z += delta_per_block) {
    for(float y = -(radius + delta_per_block); y <= (radius + delta_per_block); y += delta_per_block) {
      for(float x = -(radius + delta_per_block); x <= (radius + delta_per_block); x += delta_per_block) {


        //check whether a voxel is close enough to the box
        if( ((x - delta_per_block < -radius) && (x + delta_per_block > -radius)) ||
            ((y - delta_per_block < -radius) && (y + delta_per_block > -radius)) ||
            ((z - delta_per_block < -radius) && (z + delta_per_block > -radius)) || 

            ((x - delta_per_block < radius) && (x + delta_per_block > radius)) ||
            ((y - delta_per_block < radius) && (y + delta_per_block > radius)) ||
            ((z - delta_per_block < radius) && (z + delta_per_block > radius)) 

          ) {

          // create new point
          XYZ32F_RGBA8 point_to_push;

          // pack the 4 unsigned character values (note: alpha is just for padding purposes) into a uint32_t
          uint32_t packed_color {0};
          uint8_t r = 5.0 * radius * 255.0f;
          uint8_t g = 1.0 - (5.0 *radius * 255.0f);
          uint8_t b = std::cos(tick/1000.0) * 255.0f;

          // OR the values together, after they are shifted:    

          //    (rrrrrrrr << 24) | (gggggggg << 16) | (bbbbbbbb < 8) | (aaaaaaaa < 0) = rrrrrrrrggggggggbbbbbbbbaaaaaaaa
          //    the client takes care of the unpacking implicitly

          packed_color |= ((r << 24) | (g << 16) | (b << 8) );
          //create the point, give y a little offset (because on the client app there is a plane at the xz plane at y = 0)
          point_to_push = {x, y + 0.25f, z, packed_color};

          //add the point
          out_colored_points.push_back(point_to_push);
        }
      
      }
    }
  }

  return out_colored_points;
}

int main(int argc, char* argv[]){

  CMDParser p("socket");

  p.init(argc,argv);

  std::string socket_name(p.getArgs()[0]);


  //socket setup
  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher

  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + socket_name);
  socket.bind(endpoint.c_str());

  //fram counter
  unsigned tick = 0;

  // allocate constant size header array once. After 100 byte, the client protocol expects the XYZ32F_RGBA8 points to start
  size_t header_byte_size = 100;
  std::vector<uint8_t> header_data(header_byte_size, 0);

  while(true){

    //calculate side length of an implicit cube 
    float current_half_side_length = (std::sin(tick / 500.0) + 1.0) / 2.0;
    current_half_side_length /= 10.0f;

    // generate points on the cube
    auto points_to_send = generate_points_on_implicit_sphere(current_half_side_length, tick);

    // print number of points to send every 100 frames
    if(0 == tick % 100)
    std::cout << "Num points in points vector: " << points_to_send.size() << "\n";


    // get current number of points to send
    size_t num_points_to_send = points_to_send.size();
    // calculate corresponding byte size
    size_t size_of_points_buffer_in_byte = num_points_to_send * sizeof(XYZ32F_RGBA8);

    // prepare message containing header + point data
    zmq::message_t zmqm(header_byte_size + size_of_points_buffer_in_byte);

    // copy number of points to beginning of header (NOTE: the remaining header bytes stay empty for now)
    memcpy((char*) &header_data[0], (char*)&num_points_to_send, sizeof(num_points_to_send));

    //copy header to begin of zmq message
    memcpy( (void*) zmqm.data(), (const void*) &header_data[0], header_byte_size);
    //copy point data after the first 100 header bytes
    memcpy( (void* ) (((char*)(zmqm.data()) ) + header_byte_size) , (const void*) &points_to_send[0], size_of_points_buffer_in_byte);

    //send the data
    socket.send(zmqm);
    ++tick;
  }

  return 0;
}
