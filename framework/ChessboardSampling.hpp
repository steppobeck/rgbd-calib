#ifndef RGBD_CALIB_CHESSBOARDSAMPLING_H
#define RGBD_CALIB_CHESSBOARDSAMPLING_H

#include <DataTypes.hpp>


#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>

#include <string>
#include <vector>


  class SweepStatistics{
  public:
    unsigned input_frames;
    unsigned no_too_few_corners;
    unsigned flipped_boards;
    unsigned outliers;
    unsigned corrupt_depth;
    unsigned temporal_jitter;
    unsigned output_frames;
  };


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


  /*
    0   1  2  3  4  5  6
    7  UL  9 10 UR 12 13
    14 15 16 17 18 19 20
    21 LL 23 24 LR 26 27
    28 29 30 31 32 33 34
  */
  struct ChessboardExtremas{
    unsigned UL;
    unsigned UR;
    unsigned LL;
    unsigned LR;
  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardExtremas& v);

  class ChessboardViewRGB{
  public:
    uv corners[CB_WIDTH*CB_HEIGHT];
    float quality[CB_WIDTH*CB_HEIGHT];
    double time;
    unsigned valid;

    shape_stats calcShapeStats();
    ChessboardExtremas findExtremas();
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
    ChessboardExtremas findExtremas();
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
    ChessboardSampling(const char* filenamebase, const RGBDConfig& cfg, bool undist);
    virtual ~ChessboardSampling();

    void interactiveShow(unsigned start, unsigned end);

    bool init(bool load_poses = true);

    void dump();

    double searchSlowestTime(double starttime) const;
    glm::mat4 interpolatePose(double time, bool& valid) const;
    double getPoseSpeed(double time, bool& valid);
    ChessboardViewRGB interpolateRGB(double time, bool& valid) const;
    ChessboardViewIR interpolateIR(double time, bool& valid) const;

    const std::vector<ChessboardViewIR>& getIRs() const{
      return m_cb_ir;
    }

    const std::vector<ChessboardViewRGB>& getRGBs() const{
      return m_cb_rgb;
    }

    const std::vector<ChessboardPose>& getPoses() const{
      return m_poses;
    }

    const std::vector<ChessboardRange>& getValidRanges() const{
      return m_valid_ranges;
    }

    void filterSamples(const float pose_offset);

    bool loadRecording();

    bool loadChessboards();

    bool saveChessboards();

    double searchStartIR() const;

    std::vector<unsigned> extractBoardsForIntrinsicsFromValidRanges(const unsigned grid_w,
								    const unsigned grid_h,
								    const unsigned grid_d);
    std::vector<unsigned> getChessboardIDs();



  private:

    bool needToReload();

    bool loadPoses();
    

    bool showRecordingAndPoses(unsigned start, unsigned end);
    
    void fillCBsFromCDs(OpenCVChessboardCornerDetector* cd_rgb, OpenCVChessboardCornerDetector* cd_ir,
				     ChessboardViewRGB& cb_rgb, ChessboardViewIR& cb_ir,
			const std::vector<bool>& corner_mask_rgb, const std::vector<bool>& corner_mask_ir, float* depth);

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

    double computeCombinedBoardQuality(const unsigned cb_id);

    xyz computeAverageCornerIR(const unsigned cb_id);
    std::vector<bool> findSubBoard(OpenCVChessboardCornerDetector* cd, unsigned char* image, unsigned bytes, bool show_image, bool& success);

    float findSBAndCornerDist(OpenCVChessboardCornerDetector* cd, unsigned char* image, unsigned bytes, unsigned board_width, unsigned board_height, bool show_image);

    void processPerThread(unsigned char* rgb,
			  float* depth,
			  unsigned char* ir,
			  OpenCVChessboardCornerDetector* cd_c,
			  OpenCVChessboardCornerDetector* cd_i,
			  std::vector<unsigned>* valids,
			  const size_t frame_id,
			  const unsigned tid);
			  
  public:
    SweepStatistics p_sweep_stats;
  private:
    std::string m_filenamebase;
  protected:
    std::vector<ChessboardPose> m_poses;
    std::vector<ChessboardViewRGB> m_cb_rgb;
    std::vector<ChessboardViewIR> m_cb_ir;
    std::vector<ChessboardRange> m_valid_ranges;
    RGBDConfig m_cfg;
    bool m_undist;
  };



#endif // #ifndef RGBD_CALIB_CHESSBOARDSAMPLING_H
