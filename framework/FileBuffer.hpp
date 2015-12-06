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
	
    bool open(const char* mode = "a+", unsigned long long buffersize = 0);
    unsigned calcNumFrames(unsigned long long framesize);
    void close();

    void rewindFile();

    void setLooping(bool onoff);
    bool getLooping();
		  
    unsigned long long read (void* buffer, unsigned long long numbytes);
    unsigned long long write(void* buffer, unsigned long long numbytes);

    unsigned long long numBytesR() const;
    unsigned long long numBytesW() const;
	  
  private:
    std::string m_path;
    FILE*     m_file;
    char* m_buffer;
    unsigned long long m_bytes_r;
    unsigned long long m_bytes_w;
    struct stat m_fstat;
    bool m_looping;
  };



#endif // #ifndef RGBD_CALIB_FILEBUFFER_HPP

