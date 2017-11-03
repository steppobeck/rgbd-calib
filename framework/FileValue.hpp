#ifndef SENSOR_FILEVALUE_H
#define SENSOR_FILEVALUE_H


#include <string>

namespace sensor{

  class FileValue{

  public:
    FileValue(const char* path);
    ~FileValue();

    bool read(float& value);
    bool read(int& value);
    bool read(unsigned int& value);

  private:
    std::string m_path;
  };

}


#endif // #ifndef SENSOR_FILEVALUE_H
