
#include "sensor.hpp"

// includes, system

#include <boost/thread/mutex.hpp>

// includes, project


// internal unnamed namespace

namespace {
  
  // types, internal (class, enum, struct, union, typedef)

  // variables, internal
  
  // functions, internal

} // namespace {

// variables, exported

// functions, exported

namespace sensor {


sensor::sensor(const device* a, size_t b)
  : _station(a->getStation(b)),
    _devicemat(),
    _transmitterOffset(),
    _receiverOffset()
{}

sensor::~sensor()
{}

size_t
sensor::getFrame() const
{
  boost::mutex::scoped_lock lock(*_station.mutex);
  
  return _station.frame;
}

const timevalue&
sensor::getTimestamp() const
{
  boost::mutex::scoped_lock lock(*_station.mutex);

  return _station.timestamp;
}

const glm::mat4&
sensor::getMatrix()
{
  {
    
    boost::mutex::scoped_lock lock(*_station.mutex);
    
    _devicemat = _station.matrix;
  }
  
  _devicemat = _devicemat * _receiverOffset;
  _devicemat =  _transmitterOffset * _devicemat;
  
  return _devicemat;
}

bool
sensor::getButton(size_t a) const
{
  boost::mutex::scoped_lock lock(*_station.mutex);

  return _station.button[a];
}

void
sensor::setTransmitterOffset(const glm::mat4& a)
{
  _transmitterOffset = a;
}

void
sensor::setReceiverOffset(const glm::mat4& a)
{
  _receiverOffset = a;
}

const glm::mat4&
sensor::getTransmitterOffset() const
{
  return _transmitterOffset;
}

const glm::mat4&
sensor::getReceiverOffset() const
{
  return _receiverOffset;
}

} // end namespace sensor {
