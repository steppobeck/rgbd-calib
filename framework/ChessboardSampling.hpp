#ifndef RGBD_CALIB_CHESSBOARDSAMPLING_H
#define RGBD_CALIB_CHESSBOARDSAMPLING_H

#include <DataTypes.hpp>

#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>

#include <string>
#include <vector>


  class ChessboardRange{
  public:
    unsigned start;
    unsigned end;
    double avg_frametime;
    double sd_frametime;
    double max_frametime;
    double median_frametime;
  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardRange& v);



  class ChessboardViewRGB{
  public:
    uv corners[CB_WIDTH*CB_HEIGHT];
    float quality[CB_WIDTH*CB_HEIGHT];
    double time;
    unsigned valid;

    shape_stats calcShapeStats();
  private:
    void fillShapeIds();
    static std::vector<shape_desc> shape_descs;

  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardViewRGB& v);

  class ChessboardViewIR{
  public:
    xyz corners[CB_WIDTH*CB_HEIGHT];
    float quality[CB_WIDTH*CB_HEIGHT];
    double time;
    unsigned valid;

    shape_stats calcShapeStats();
  private:
    void fillShapeIds();
    static std::vector<shape_desc> shape_descs;

  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardViewIR& v);

  class ChessboardPose{
  public:
    double time;
    glm::mat4 mat;
  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardPose& v);


  class OpenCVChessboardCornerDetector;

  class ChessboardSampling{

  public:
    ChessboardSampling(const char* filenamebase);
    virtual ~ChessboardSampling();

    void interactiveShow(unsigned start, unsigned end);

    bool init();

    void dump();

    double searchSlowestTime(double starttime) const;
    glm::mat4 interpolatePose(double time, bool& valid) const;
    double getPoseSpeed(double time, bool& valid);
    ChessboardViewRGB interpolateRGB(double time, bool& valid) const;
    ChessboardViewIR interpolateIR(double time, bool& valid) const;

    const std::vector<ChessboardViewIR>& getIRs() const{
      return m_cb_ir;
    }


    const std::vector<ChessboardPose>& getPoses() const{
      return m_poses;
    }

    const std::vector<ChessboardRange>& getValidRanges() const{
      return m_valid_ranges;
    }

    void filterSamples(const float pose_offset);

    bool loadRecording();
    bool loadRecordingSeq();

    bool loadChessboards();

    bool saveChessboards();

    double searchStartIR() const;

  private:

    bool needToReload();

    bool loadPoses();
    

    bool showRecordingAndPoses(unsigned start, unsigned end);
    
    

    void detectFlips();

    float computeAVGRGBFrequency();
    float computeAVGIRFrequency();

    void detectCorruptedDepthInRanges();

    void detectShapeFaultsInRanges();

    void calcStatsInRanges();

    void gatherValidRanges();

    void detectTimeJumpsInRanges();

    void gatherCornerTracesInRanges(const char* prefix);


    void oneEuroFilterInRanges();

    void computeQualityFromSpeedIRInRanges(const float pose_offset);

    void computeCornerQualityInRanges();

    void invalidateAt(unsigned cb_id, unsigned window_size);


    void processPerThread(unsigned char* rgb,
			  float* depth,
			  unsigned char* ir,
			  OpenCVChessboardCornerDetector* cd_c,
			  OpenCVChessboardCornerDetector* cd_i,
			  std::vector<unsigned>* valids,
			  const size_t frame_id,
			  const unsigned tid);
			  


    std::string m_filenamebase;
  protected:
    std::vector<ChessboardPose> m_poses;
    std::vector<ChessboardViewRGB> m_cb_rgb;
    std::vector<ChessboardViewIR> m_cb_ir;
    std::vector<ChessboardRange> m_valid_ranges;
  };



#endif // #ifndef RGBD_CALIB_CHESSBOARDSAMPLING_H
