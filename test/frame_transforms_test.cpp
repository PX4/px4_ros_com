/****************************************************************************
 *
 * Copyright 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include <px4_ros_com/frame_transforms.h>

namespace px4_ros_com
{
namespace frame_transforms
{

TEST(FrameTransforms, RotatesCovariance3d)
{
  const Covariance3d covariance = {
    4.0, 1.0, -0.5,
    1.0, 2.0, 0.25,
    -0.5, 0.25, 1.0
  };
  const Eigen::Quaterniond q =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
  const EigenMapConstCovariance3d covariance_matrix(covariance.data());
  const Eigen::Matrix3d expected = rotation * covariance_matrix * rotation.transpose();

  const Covariance3d transformed = transform_frame(covariance, q);
  const EigenMapConstCovariance3d transformed_matrix(transformed.data());

  EXPECT_TRUE(transformed_matrix.isApprox(expected, 1e-12));
}

}  // namespace frame_transforms
}  // namespace px4_ros_com
