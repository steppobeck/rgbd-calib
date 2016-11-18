#include <ChessboardSampling.hpp>
#include <iostream>

int main(int argc, char** argv){


  ChessboardViewIR c;

   c.quality[0]=0.0 ;c.quality[1]=0.0 ;c.quality[2]=0.0; c.quality[3]=0.0 ;c.quality[4]=0.0; c.quality[5]=0.0; c.quality[6]=0.0;

   c.quality[7]=0.0 ;c.quality[8]=1.0 ;c.quality[9]=1.0;c.quality[10]=1.0;c.quality[11]=1.0;c.quality[12]=1.0;c.quality[13]=1.0;

  c.quality[14]=0.0;c.quality[15]=1.0;c.quality[16]=1.0;c.quality[17]=1.0;c.quality[18]=1.0;c.quality[19]=1.0;c.quality[20]=1.0;

  c.quality[21]=0.0;c.quality[22]=1.0;c.quality[23]=1.0;c.quality[24]=1.0;c.quality[25]=1.0;c.quality[26]=1.0;c.quality[27]=1.0;

  c.quality[28]=0.0;c.quality[29]=0.0;c.quality[30]=0.0;c.quality[31]=0.0;c.quality[32]=0.0;c.quality[33]=0.0;c.quality[34]=0.0;


  ChessboardExtremas e = c.findExtremas();

  std::cout << e << std::endl;


  return 0;
}

