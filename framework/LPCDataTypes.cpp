#include "LPCDataTypes.hpp"
#include <cmath>

std::ostream& operator << (std::ostream& o, const glm::dvec3& v){
  o << "(" << v.x << "," << v.y << "," << v.z << ")";
  return o;
}



const float Gamma        =   0.80;
const float IntensityMax = 255;
	
double round(double d){
  return floor(d + 0.5);
}
	
unsigned char Adjust(const float Color, const float Factor){
  if (Color == 0.0){
    return 0;
  }
  else{
    int res = round(IntensityMax * std::pow(Color * Factor, Gamma));
    return std::min(255, std::max(0, res));
  }
}

void WavelengthToRGB(const float Wavelength, unsigned char& R, unsigned char& G, unsigned char& B){
  double Blue;
  double factor;
  double Green;
  double Red;
  if(380 <= Wavelength && Wavelength <= 440){
    Red   = -(Wavelength - 440) / (440 - 380);
    Green = 0.0;
    Blue  = 1.0;
  }
  else if(440 < Wavelength && Wavelength <= 490){
    Red   = 0.0;
    Green = (Wavelength - 440) / (490 - 440);
    Blue  = 1.0;
  }
  else if(490 < Wavelength && Wavelength <= 510){
    Red   = 0.0;
    Green = 1.0;
    Blue  = -(Wavelength - 510) / (510 - 490);
  }
  else if(510 < Wavelength && Wavelength <= 580){
    Red   = (Wavelength - 510) / (580 - 510);
    Green = 1.0;
    Blue  = 0.0;
  }
  else if(580 < Wavelength && Wavelength <= 645){		
    Red   = 1.0;
    Green = -(Wavelength - 645) / (645 - 580);
    Blue  = 0.0;
  }
  else if(645 < Wavelength && Wavelength <= 780){
    Red   = 1.0;
    Green = 0.0;
    Blue  = 0.0;
  }
  else{
    Red   = 0.0;
    Green = 0.0;
    Blue  = 0.0;
  }
		
		
  if(380 <= Wavelength && Wavelength <= 420){
    factor = 0.3 + 0.7*(Wavelength - 380) / (420 - 380);
  }
  else if(420 < Wavelength && Wavelength <= 701){
    factor = 1.0;
  }
  else if(701 < Wavelength && Wavelength <= 780){
    factor = 0.3 + 0.7*(780 - Wavelength) / (780 - 701);
  }
  else{
    factor = 0.0;
  }
  R = Adjust(Red,   factor);
  G = Adjust(Green, factor);
  B = Adjust(Blue,  factor);
}
	
	
	

float GetWaveLengthFromDataPoint(float Value, float MinValue, float MaxValue){
  const float MinVisibleWavelength = 380.0;//350.0;
  const float MaxVisibleWavelength = 780.0;//650.0;
  //Convert data value in the range of MinValues..MaxValues to the 
  //range 350..650
  return (Value - MinValue) / (MaxValue-MinValue) * (MaxVisibleWavelength - MinVisibleWavelength) + MinVisibleWavelength;
}	

	
glm::uvec3 DataPointToColor(float Value, float MinValue, float MaxValue){
  unsigned char r = 0;
  unsigned char g = 0;
  unsigned char b = 0;
  float Wavelength;
  
  Wavelength = GetWaveLengthFromDataPoint(Value, MinValue, MaxValue);
  WavelengthToRGB(Wavelength, r, g, b);	  
  
  glm::uvec3 result;
  result[0] = r;
  result[1] = g;
  result[2] = b;
  return result;
}


glm::uvec3 DataPointToColorSeismic(float Value, float MinValue, float MaxValue){
  unsigned char r = 255;
  unsigned char g = 255;
  unsigned char b = 255;
    

  if(0.0 > Value){
    float fac = std::abs(Value)/std::abs(MinValue);
    r = 255 - (unsigned char) 255.0 * fac;
    g = 255 - (unsigned char) 255.0 * fac;
  }
  else if(0.0 < Value){
    float fac = std::abs(Value)/std::abs(MaxValue);
    g = 255 - (unsigned char) 255.0 * fac;
    b = 255 -(unsigned char) 255.0 * fac;
  }
  
  glm::uvec3 result;
  result[0] = r;
  result[1] = g;
  result[2] = b;
  return result;
}
