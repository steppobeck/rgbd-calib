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
	
    bool open(const char* mode = "a+", unsigned buffersize = 0);
    unsigned calcNumFrames(unsigned framesize);
    void close();

    void rewindFile();

    void setLooping(bool onoff);
    bool getLooping();
		  
    unsigned read (void* buffer, unsigned numbytes);
    unsigned write(void* buffer, unsigned numbytes);

    unsigned numBytesR() const;
    unsigned numBytesW() const;
	  
  private:
    std::string m_path;
    FILE*     m_file;
    char* m_buffer;
    unsigned  m_bytes_r;
    unsigned  m_bytes_w;
    struct stat m_fstat;
    bool m_looping;
  };



#endif // #ifndef RGBD_CALIB_FILEBUFFER_HPP

