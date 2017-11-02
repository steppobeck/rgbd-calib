#include <CMDParser.hpp>
#include <FileBuffer.hpp>
#include <squish.h>

int main(int argc, char* argv[]){
  //unsigned num_kinect_cameras = 1;
  CMDParser p("infile.stream outfile.stream");
  //p.addOpt("k",1,"num_kinect_cameras", "specify how many kinect cameras are in stream, default: 1");
  p.init(argc,argv);
  /*
    if(p.isOptSet("k")){
    num_kinect_cameras = p.getOptsInt("k")[0];
    }
  */

  if(p.getArgs().size() != 2){
    p.showHelp();
    return 0;
  }

  FileBuffer infile(p.getArgs()[0].c_str());
  infile.open("r");
  FileBuffer outfile(p.getArgs()[1].c_str());
  outfile.open("w",0);

  const unsigned color_width = 1280;
  const unsigned color_height = 1080;

  const unsigned in_colorsize = 691200;
  const unsigned out_colorsize = color_width * color_height * 3;
  const unsigned tmp_colorsize = color_width * color_height * 4;
  const unsigned depthsize = 512 * 424 * sizeof(float);
  const size_t in_frame_size_bytes(in_colorsize + depthsize);
  
  unsigned char* in_color_buff = new unsigned char[in_colorsize];
  unsigned char* out_color_buff = new unsigned char[out_colorsize];
  unsigned char* tmp_color_buff = new unsigned char[tmp_colorsize];
  unsigned char* depth_buff = new unsigned char[depthsize];

  const unsigned num_frames = infile.getFileSizeBytes()/in_frame_size_bytes;

  for(unsigned i = 0; i < num_frames; ++i){
    infile.read(in_color_buff, in_colorsize);
    infile.read(depth_buff, depthsize);

    squish::DecompressImage (tmp_color_buff, color_width, color_height, in_color_buff, squish::kDxt1);
    unsigned buffida = 0;
    unsigned buffid = 0;
    for(unsigned y = 0; y < color_height; ++y){
      for(unsigned x = 0; x < color_width; ++x){
	out_color_buff[buffid++] = tmp_color_buff[buffida++];
	out_color_buff[buffid++] = tmp_color_buff[buffida++];
	out_color_buff[buffid++] = tmp_color_buff[buffida++];
	buffida++;
      }
    }

    outfile.write(out_color_buff, out_colorsize);
    outfile.write(depth_buff, depthsize);
  }

  infile.close();
  outfile.close();

  delete [] in_color_buff;
  delete [] out_color_buff;
  delete [] tmp_color_buff;
  delete [] depth_buff;

  return 0;
}
