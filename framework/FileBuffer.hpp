#ifndef RGBD_CALIB_FILEBUFFER_HPP
#define RGBD_CALIB_FILEBUFFER_HPP

#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>


  class FileBuffer{

  public:
    FileBuffer(const char* path);
    ~FileBuffer();

    bool isOpen();
	
    bool open(const char* mode = "a+", size_t buffersize = 0);
    unsigned calcNumFrames(size_t framesize);
    void close();

    void rewindFile();

    void setLooping(bool onoff);
    bool getLooping();
    size_t getFileSizeBytes();
    void gotoByte(size_t offset);
  
    size_t read (void* buffer, size_t numbytes);
    size_t write(void* buffer, size_t numbytes);

    size_t numBytesR() const;
    size_t numBytesW() const;
	  
  private:
    std::string m_path;
    FILE*     m_file;
    char* m_buffer;
    size_t m_bytes_r;
    size_t m_bytes_w;
    struct stat m_fstat;
    bool m_looping;
  };



#endif // #ifndef RGBD_CALIB_FILEBUFFER_HPP

