#include "CSVExporter.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>

CSVExporter::CSVExporter(unsigned numslots)
  : slotnames(),
    m_slots()
{
  for(unsigned i = 0; i != numslots; ++i){
    m_slots.push_back(std::vector<double>());
  }
}


CSVExporter::~CSVExporter()
{}



void
CSVExporter::push(unsigned slot, double value){
  m_slots[slot].push_back(value);
}


void
CSVExporter::save(const char* filename){


  std::ofstream f(filename);
  
  for(unsigned s = 0; s != slotnames.size(); ++s){
    f << slotnames[s];
    if(s == (slotnames.size() - 1))
      f << std::endl;
    else
      f << ";";
  }

  for(unsigned v = 0; v != m_slots[0].size(); ++v){
    for(unsigned s = 0; s != m_slots.size(); ++s){
      f << std::setprecision(15) << m_slots[s][v];
      if(s == (m_slots.size() - 1))
	f << std::endl;
      else
	f << ";";
    }
  }

}
