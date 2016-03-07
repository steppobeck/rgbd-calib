#include <ChessboardSampling.hpp>

int main(void){

  ChessboardSampling cbs("/tmp/23_sweep");

  cbs.loadRecordingSeq();
  cbs.dump();
#if 0
  cbs.saveChessboards();
  system("mv /tmp/23_sweep.chessboardsir /tmp/23_sweep.chessboardsir_parallel");
  system("mv /tmp/23_sweep.chessboardsrgb /tmp/23_sweep.chessboardsrgb_parallel");

  cbs.loadRecordingSeq();
  cbs.dump();
  cbs.saveChessboards();

  system("diff /tmp/23_sweep.chessboardsir /tmp/23_sweep.chessboardsir_parallel");
  system("diff /tmp/23_sweep.chessboardsrgb /tmp/23_sweep.chessboardsrgb_parallel");
#endif
  return 0;
}

