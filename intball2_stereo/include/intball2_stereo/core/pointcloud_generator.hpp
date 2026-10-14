#ifndef INTBALL2_STEREO_CORE_POINTCLOUD_GENERATOR_HPP_
#define INTBALL2_STEREO_CORE_POINTCLOUD_GENERATOR_HPP_

#include "intball2_stereo/core/stereo_config.hpp"

namespace intball2_stereo::core
{
class PointCloudGenerator
{
public:
  // Owns points independently of the processor's reusable depth buffer.
  static PointCloudResult generate(const DepthResult &depth, const PointsConfig &config);
};
} // namespace intball2_stereo::core

#endif // INTBALL2_STEREO_CORE_POINTCLOUD_GENERATOR_HPP_
