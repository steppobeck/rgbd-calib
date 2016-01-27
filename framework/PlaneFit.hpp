#ifndef RGBD_CALIB_PLANEFIT_HPP
#define RGBD_CALIB_PLANEFIT_HPP


#include <DataTypes.hpp>

#include <vector>



double detectPlaneQuality(const std::vector<xyz>& corners);

#endif // #ifndef RGBD_CALIB_PLANEFIT_HPP
