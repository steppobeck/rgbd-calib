#include <ChessboardSampling.hpp>

int main(void){

  ChessboardSampling cbs("/tmp/23_sweep");

  cbs.loadRecording();
  cbs.saveChessboards();
  system("mv /tmp/23_sweep.chessboardsir /tmp/23_sweep.chessboardsir_parallel");
  system("mv /tmp/23_sweep.chessboardsrgb /tmp/23_sweep.chessboardsrgb_parallel");

  cbs.loadRecordingSeq();
  cbs.saveChessboards();


  return 0;
}

