#ifndef RGBD_CALIB_PLANEFIT_HPP
#define RGBD_CALIB_PLANEFIT_HPP


#include <DataTypes.hpp>

#include <vector>



float detectPlaneQuality(const std::vector<xyz>& corners);

#endif // #ifndef RGBD_CALIB_PLANEFIT_HPP
