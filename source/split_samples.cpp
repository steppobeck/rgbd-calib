#include <DataTypes.hpp>
#include <CMDParser.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <algorithm> // std::shuffle
#include <fstream>
#include <iostream>
#include <unistd.h>


class RSBoard{


public:

  void read(std::ifstream& iff, float& min_depth_sample, float& max_depth_sample){
    
    for(unsigned i = 0; i < CB_WIDTH*CB_HEIGHT; ++i){
      samplePoint s;
      iff.read((char*) &s.depth, sizeof(float));
      iff.read((char*) &s.tex_color, sizeof(uv));
      iff.read((char*) &s.tex_depth, sizeof(uv));
      iff.read((char*) &s.pos_offset, sizeof(xyz));
      iff.read((char*) &s.tex_offset, sizeof(uv));
      iff.read((char*) glm::value_ptr(s.pos_real), sizeof(glm::vec3));
      iff.read((char*) &s.quality, sizeof(float));

      min_depth_sample = std::min(s.depth, min_depth_sample);
      max_depth_sample = std::max(s.depth, max_depth_sample);

      ss[i] = s;
    }
  }

  void write(std::ofstream* off){
    for(unsigned i = 0; i < CB_WIDTH*CB_HEIGHT; ++i){
      samplePoint s = ss[i];
      off->write((const char*) &s.depth, sizeof(float));
      off->write((const char*) &s.tex_color, sizeof(uv));
      off->write((const char*) &s.tex_depth, sizeof(uv));
      off->write((const char*) &s.pos_offset, sizeof(xyz));
      off->write((const char*) &s.tex_offset, sizeof(uv));
      off->write((const char*) glm::value_ptr(s.pos_real), sizeof(glm::vec3));
      off->write((const char*) &s.quality, sizeof(float));
    }
  }

  glm::vec3 getPos(){
    return ss[0].pos_real;
  }

  samplePoint ss[CB_WIDTH*CB_HEIGHT];

};


int main(int argc, char* argv[]){
  bool shuffle = false;
  float min_depth_sample = std::numeric_limits<float>::max();
  float max_depth_sample = std::numeric_limits<float>::lowest();
  CMDParser p("input outputA outputB");
  p.addOpt("s",-1,"shuffle", "shuffle and split, default: false");
  p.init(argc,argv);

  if(p.getArgs().size() != 3){
    p.showHelp();
  }

  if(p.isOptSet("s")){
    shuffle = true;
  }


  
  std::ifstream iff(p.getArgs()[0].c_str(), std::ifstream::binary);
  std::ofstream off1(p.getArgs()[1].c_str(), std::ifstream::binary);
  std::ofstream off2(p.getArgs()[2].c_str(), std::ifstream::binary);
  std::ofstream* off = 0;
  const unsigned num_samples_in_file = calcNumFrames(iff,
						     sizeof(float) +
						     sizeof(uv) +
						     sizeof(uv) +
						     sizeof(xyz) +
						     sizeof(uv) +
						     sizeof(glm::vec3) +
						     sizeof(float));

  // two methods
  if(shuffle){

    std::vector<samplePoint> sps;
    for(unsigned i = 0; i < num_samples_in_file; ++i){

      samplePoint s;
      iff.read((char*) &s.depth, sizeof(float));
      iff.read((char*) &s.tex_color, sizeof(uv));
      iff.read((char*) &s.tex_depth, sizeof(uv));
      iff.read((char*) &s.pos_offset, sizeof(xyz));
      iff.read((char*) &s.tex_offset, sizeof(uv));
      iff.read((char*) glm::value_ptr(s.pos_real), sizeof(glm::vec3));
      iff.read((char*) &s.quality, sizeof(float));
      
      min_depth_sample = std::min(s.depth, min_depth_sample);
      max_depth_sample = std::max(s.depth, max_depth_sample);

      sps.push_back(s);
    }

    std::shuffle(std::begin(sps), std::end(sps), std::default_random_engine());
    
    bool to_off1 = false;
    for(const samplePoint& s : sps){
      
      to_off1 = !to_off1;
      
      

      off = to_off1 ? &off1 : &off2;
      
      off->write((const char*) &s.depth, sizeof(float));
      off->write((const char*) &s.tex_color, sizeof(uv));
      off->write((const char*) &s.tex_depth, sizeof(uv));
      off->write((const char*) &s.pos_offset, sizeof(xyz));
      off->write((const char*) &s.tex_offset, sizeof(uv));
      off->write((const char*) glm::value_ptr(s.pos_real), sizeof(glm::vec3));
      off->write((const char*) &s.quality, sizeof(float));

      std::cout << "written to " << (to_off1 ? p.getArgs()[1] : p.getArgs()[2]) << std::endl;

    }

  }
  else{
    // intermediate board locations go to off2, others are used for calibration and go to off1
    const unsigned num_boards = num_samples_in_file / (CB_WIDTH*CB_HEIGHT);
    std::cout << "going to split " << num_boards <<  " into intermediate locations" << std::endl;
    // 1. read boards and track wether pos_real of first sample changed

    std::vector< std::vector<RSBoard> > board_seqs;
    
    
    bool first = true;
    glm::vec3 pos_real;
    glm::vec3 last_pos_real;
    for(unsigned bid = 0 ; bid < num_boards; ++bid){

      // read board
      RSBoard board;
      board.read(iff, min_depth_sample, max_depth_sample);
      pos_real = board.getPos();

      if(first){
	last_pos_real = pos_real;
	first = false;
	board_seqs.push_back(std::vector<RSBoard>());
      }
      const float dist_to_last_board = glm::length(pos_real - last_pos_real);
      if(dist_to_last_board > 0.5 /*meter*/){
	std::cout << "switch in sequence detected: " << dist_to_last_board << std::endl;
	board_seqs.push_back(std::vector<RSBoard>());
      }
      last_pos_real = pos_real;

      board_seqs.back().push_back(board);
    }

    // put into off1 or off2
    for(const auto& s : board_seqs){
      const unsigned num_in_seq = s.size();
      std::cout << "-------------------> start splitting seq size: " << num_in_seq << std::endl;
      bool to_off1 = false;
      for(unsigned i = 0; i < num_in_seq; ++i){
	to_off1 = !to_off1;
	off = to_off1 ? &off1 : &off2;

	if(!to_off1 && (num_in_seq - i) == 1){
	  break;
	}
	RSBoard board = s[i];
	board.write(off);

	std::cout << "written to " << (to_off1 ? p.getArgs()[1] : p.getArgs()[2]) << std::endl;

      }
    }

  }
  iff.close();
  off1.close();
  off2.close();

  std::cout << "min_depth: " << min_depth_sample << std::endl;
  std::cout << "max_depth: " << max_depth_sample << std::endl;

  return 0;
}
