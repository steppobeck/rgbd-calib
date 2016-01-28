#ifndef RGBD_CALIB_CSVEXPORTER_HPP
#define RGBD_CALIB_CSVEXPORTER_HPP



#include <string>
#include <vector>

class CSVExporter{

public:
  CSVExporter(unsigned numslots);
  ~CSVExporter();
  std::vector<std::string> slotnames;

  void push(unsigned slot, double value);
  void save(const char* filename);


private:


  std::vector<std::vector<double> > m_slots;

};



#endif // #ifndef RGBD_CALIB_CSVEXPERTER_HPP


