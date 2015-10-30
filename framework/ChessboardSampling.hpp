#ifndef RGBD_CALIB_CHESSBOARDSAMPLING_H
#define RGBD_CALIB_CHESSBOARDSAMPLING_H

#include <DataTypes.hpp>
#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>

#include <string>
#include <vector>



  class ChessboardViewRGB{
  public:
    uv corners[CB_WIDTH*CB_HEIGHT];
    float quality[CB_WIDTH*CB_HEIGHT];
    double time;
  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardViewRGB& v);

  class ChessboardViewIR{
  public:
    xyz corners[CB_WIDTH*CB_HEIGHT];
    float quality[CB_WIDTH*CB_HEIGHT];
    double time;
  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardViewIR& v);

  class ChessboardPose{
  public:
    double time;
    glm::mat4 mat;
  };

  extern std::ostream& operator << (std::ostream& o, const ChessboardPose& v);


  class ChessboardSampling{

  public:
    ChessboardSampling(const char* filenamebase);
    virtual ~ChessboardSampling();

    void interactiveShow();

    bool init(bool reload = true);

    void dump();

    double searchSlowestTime(double starttime);
    glm::mat4 interpolatePose(double time, bool& valid);
    double getPoseSpeed(double time, bool& valid);
    ChessboardViewRGB interpolateRGB(double time, bool& valid);
    ChessboardViewIR interpolateIR(double time, bool& valid);

    std::vector<ChessboardViewIR>& getIRs(){
      return m_cb_ir;
    }

    void filterIR(float pose_offset);

    bool saveChessboards();

    double searchStartIR();

  private:
    bool loadPoses();
    bool loadRecording();

    bool showRecordingAndPoses();
    
    bool loadChessboards();


    void computeQualityIR(float pose_offset);

    std::string m_filenamebase;
  protected:
    std::vector<ChessboardPose> m_poses;
    std::vector<ChessboardViewRGB> m_cb_rgb;
    std::vector<ChessboardViewIR> m_cb_ir;
  };



#endif // #ifndef RGBD_CALIB_CHESSBOARDSAMPLING_H
