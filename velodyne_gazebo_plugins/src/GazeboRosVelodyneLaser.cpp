/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015-2018, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <velodyne_gazebo_plugins/GazeboRosVelodyneLaser.h>

#include <algorithm>
#include <iterator>
#include <sstream>
#include <vector>
#include <string>

#include <ignition/math/Angle.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/physics/MultiRayShape.hh>

#include <gazebo/sensors/GpuRaySensor.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyneLaser)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVelodyneLaser::GazeboRosVelodyneLaser() : min_range_(0), max_range_(0), gaussian_noise_(0)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVelodyneLaser::~GazeboRosVelodyneLaser()
{
}

std::vector<GazeboRosVelodyneLaser::ScanPattern::Sample>
GazeboRosVelodyneLaser::SampleAngleIntervalEvenly(ignition::math::Angle _min_angle,
                                                  ignition::math::Angle _max_angle,
                                                  int _sample_count) {
  std::vector<GazeboRosVelodyneLaser::ScanPattern::Sample> samples;
  samples.reserve(_sample_count);
  if (_sample_count > 1) {
    ignition::math::Angle angle_step = (_max_angle - _min_angle) / (_sample_count - 1);
    for (int i = 0; i < _sample_count; ++i) {
      samples.emplace_back(i, angle_step * i + _min_angle);
    }
  } else {
    samples.emplace_back(0, 0.0);
  }
  return samples;
}

std::vector<GazeboRosVelodyneLaser::ScanPattern::Sample>
GazeboRosVelodyneLaser::SampleAngleInterval(sdf::ElementPtr _sdf,
                                            ignition::math::Angle _min_angle,
                                            ignition::math::Angle _max_angle,
                                            int _sample_count) {
  std::vector<GazeboRosVelodyneLaser::ScanPattern::Sample> samples;
  samples.reserve(_sample_count);
  if (_sdf->HasElement("angles")) {
    size_t index = 0;
    std::istringstream iss(_sdf->GetElement("angles")->Get<std::string>());
    std::transform(std::istream_iterator<double>(iss), std::istream_iterator<double>(),
                   std::back_inserter(samples), [&index] (double angle) {
                     return GazeboRosVelodyneLaser::ScanPattern::Sample(index++, angle);
                   });
    if (static_cast<int>(samples.size()) == _sample_count) {
      auto it = std::find_if(samples.begin(), samples.end(),
                             [_min_angle, _max_angle] (const GazeboRosVelodyneLaser::ScanPattern::Sample & sample) {
                               const ignition::math::Angle kEpsilonAngle{1e-3};
                               return ((sample.angle < _min_angle - kEpsilonAngle) ||
                                       (sample.angle > _max_angle + kEpsilonAngle));
                             });
      if (it == samples.end()) {
        return samples;
      } else {
        RCLCPP_WARN(ros_node_->get_logger(),
                    "Velodyne laser plugin given a %s scan angle out of bounds: "
                    "%.16g (%.16g deg) < %.16g rad (%.16g deg) < %.16g (%.16g deg), ignoring setting.",
                    _min_angle.Radian(), _min_angle.Degree(), it->angle.Radian(), it->angle.Degree(),
                    _max_angle.Radian(), _max_angle.Degree(), _sdf->GetName().c_str());
      }
    } else {
      RCLCPP_WARN(ros_node_->get_logger(),
                  "Velodyne laser plugin not given %d %s scan angles, ignoring setting.",
                  _sample_count, _sdf->GetName().c_str());
    }
  } else {
    RCLCPP_WARN(ros_node_->get_logger(),
                "Velodyne laser plugin not given any %s scan pattern, ignoring setting.",
                _sdf->GetName().c_str());
  }
  samples = SampleAngleIntervalEvenly(_min_angle, _max_angle, _sample_count);

  return samples;
}

void GazeboRosVelodyneLaser::LoadScanPattern(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
#if GAZEBO_MAJOR_VERSION >= 7
  sensors::RaySensorPtr parent_ray_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#else
  sensors::RaySensorPtr parent_ray_sensor = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
#endif
 if (parent_ray_sensor) {
  if (_sdf->HasElement("scan")) {
    sdf::ElementPtr scan_elem = _sdf->GetElement("scan");

    physics::MultiRayShapePtr laser = parent_ray_sensor->LaserShape();

    if (scan_elem->HasElement("horizontal")) {
      if (laser->GetScanResolution() == 1.) {
        scan_pattern_.horizontal_samples = SampleAngleInterval(
          scan_elem->GetElement("horizontal"), parent_ray_sensor->AngleMin(),
          parent_ray_sensor->AngleMax(), parent_ray_sensor->RangeCount());
      } else {
        RCLCPP_WARN(ros_node_->get_logger(),
                    "Velodyne laser plugin does not support uneven horizontal sampling "
                    "at non-unit resolutions for CPU ray sensors, ignoring setting");
        scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
          parent_ray_sensor->AngleMin(), parent_ray_sensor->AngleMax(), parent_ray_sensor->RangeCount());
      }
    } else {
      scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
        parent_ray_sensor->AngleMin(), parent_ray_sensor->AngleMax(), parent_ray_sensor->RangeCount());
    }

    if (scan_elem->HasElement("vertical")) {
      if (laser->GetVerticalScanResolution() == 1.) {
        scan_pattern_.vertical_samples = SampleAngleInterval(
          scan_elem->GetElement("vertical"), parent_ray_sensor->VerticalAngleMin(),
          parent_ray_sensor->VerticalAngleMax(), parent_ray_sensor->VerticalRangeCount());
      } else {
        RCLCPP_WARN(ros_node_->get_logger(),
                    "Velodyne laser plugin does not support uneven vertical sampling "
                    "at non-unit resolutions for CPU ray sensors, ignoring setting");
        scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
          parent_ray_sensor->VerticalAngleMin(), parent_ray_sensor->VerticalAngleMax(),
          parent_ray_sensor->VerticalRangeCount());
      }
    } else {
      scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
        parent_ray_sensor->VerticalAngleMin(), parent_ray_sensor->VerticalAngleMax(),
        parent_ray_sensor->VerticalRangeCount());
    }

    const double ray_min_range = laser->GetMinRange();
    const double ray_max_range = laser->GetMaxRange();
    const ignition::math::Pose3d pose = parent_ray_sensor->Pose();
    for (size_t j = 0; j < scan_pattern_.vertical_samples.size(); ++j) {
      for (size_t i = 0; i < scan_pattern_.horizontal_samples.size(); ++i) {
        const ignition::math::Quaterniond ray_orientation =
          ignition::math::Quaterniond::EulerToQuaternion(
             ignition::math::Vector3d(0.0, -scan_pattern_.vertical_samples[j].angle.Radian(),
                                      scan_pattern_.horizontal_samples[i].angle.Radian()));
        const ignition::math::Vector3d ray_axis =
          pose.Rot() * ray_orientation * ignition::math::Vector3d::UnitX;
        const size_t ray_index = scan_pattern_.horizontal_samples[i].index +
          scan_pattern_.vertical_samples[j].index *
          scan_pattern_.horizontal_samples.size();
        laser->Ray(ray_index)->SetPoints(
          ray_axis * ray_min_range + pose.Pos(),
          ray_axis * ray_max_range + pose.Pos());
      }
    }
  } else {

    scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
      parent_ray_sensor->VerticalAngleMin(),
      parent_ray_sensor->VerticalAngleMax(),
      parent_ray_sensor->VerticalRangeCount());
    scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
      parent_ray_sensor->AngleMin(), parent_ray_sensor->AngleMax(), parent_ray_sensor->RangeCount());
  }

  return;
 }

#if GAZEBO_MAJOR_VERSION >= 7
 sensors::GpuRaySensorPtr parent_gpu_ray_sensor = std::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);
#else
 sensors::GpuRaySensorPtr parent_gpu_ray_sensor = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(_parent);
#endif

 if (parent_gpu_ray_sensor) {
   if (_sdf->HasElement("scan")) {
     RCLCPP_WARN(ros_node_->get_logger(),
                 "Velodyne laser plugin does not support scan pattern "
                 "specifications for GPU ray sensors, ignoring setting");
   }
   scan_pattern_.vertical_samples = SampleAngleIntervalEvenly(
     parent_gpu_ray_sensor->VerticalAngleMin(),
     parent_gpu_ray_sensor->VerticalAngleMax(),
     parent_gpu_ray_sensor->VerticalRangeCount());
   scan_pattern_.horizontal_samples = SampleAngleIntervalEvenly(
     parent_gpu_ray_sensor->AngleMin(),
     parent_gpu_ray_sensor->AngleMax(),
     parent_gpu_ray_sensor->RangeCount());

   return;
 }

 gzthrow("GazeboRosVelodyneLaser controller requires either a Ray Sensor or a GPU Ray Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVelodyneLaser::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  gzdbg << "Loading GazeboRosVelodyneLaser\n";

  // Initialize Gazebo node
  gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gazebo_node_->Init();

  // Create node handle
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get the parent sensor
  parent_sensor_ = _parent;

  robot_namespace_ = "/";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  if (!_sdf->HasElement("frameName")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <frameName>, defaults to /world");
    frame_name_ = "/world";
  } else {
    frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("min_range")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <min_range>, defaults to 0");
    min_range_ = 0;
  } else {
    min_range_ = _sdf->GetElement("min_range")->Get<double>();
  }

  if (!_sdf->HasElement("max_range")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <max_range>, defaults to infinity");
    max_range_ = INFINITY;
  } else {
    max_range_ = _sdf->GetElement("max_range")->Get<double>();
  }

  min_intensity_ = std::numeric_limits<double>::lowest();
  if (!_sdf->HasElement("min_intensity")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <min_intensity>, defaults to no clipping");
  } else {
    min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();
  }

  LoadScanPattern(parent_sensor_, _sdf);

  if (!_sdf->HasElement("topicName")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <topicName>, defaults to /points");
    topic_name_ = "/points";
  } else {
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  }

  if (!_sdf->HasElement("gaussianNoise")) {
    RCLCPP_INFO(ros_node_->get_logger(), "Velodyne laser plugin missing <gaussianNoise>, defaults to 0.0");
    gaussian_noise_ = 0;
  } else {
    gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
  }

  if (topic_name_ != "") {
    pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic_name_, 10);
  }

  // TODO lazy subscribe. Find a way to subscribe to the gazebo topic if there are
  //      ros subscribers present.
  sub_ = gazebo_node_->Subscribe(parent_sensor_->Topic(), &GazeboRosVelodyneLaser::OnScan, this);

  RCLCPP_INFO(ros_node_->get_logger(), "Velodyne %slaser plugin ready");
  gzdbg << "GazeboRosVelodyneLaser LOADED\n";
}

void GazeboRosVelodyneLaser::OnScan(ConstLaserScanStampedPtr& _msg)
{
#if GAZEBO_MAJOR_VERSION >= 7
  const double maxRange = _msg->scan().range_max();
  const double minRange = _msg->scan().range_min();

  const int rangeCount = _msg->scan().count();
  const int verticalRangeCount = _msg->scan().vertical_count();
#else
  const double maxRange = _msg->scan().range_max();
  const double minRange = _msg->scan().range_min();

  const int rangeCount =  _msg->scan().count();
  const int verticalRangeCount = _msg->scan().vertical_count();
#endif
  // Ensure these assumptions hold.
  assert(scan_pattern_.horizontal_samples.size() <= static_cast<size_t>(rangeCount));
  assert(scan_pattern_.vertical_samples.size() <= static_cast<size_t>(verticalRangeCount));

  const double MIN_RANGE = std::max(min_range_, minRange);
  const double MAX_RANGE = std::min(max_range_, maxRange);
  const double MIN_INTENSITY = min_intensity_;

  // Populate message fields
  const uint32_t POINT_STEP = 32;
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_name_;
  msg.header.stamp.sec = _msg->time().sec();
  msg.header.stamp.nanosec = _msg->time().nsec();
  msg.fields.resize(5);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 16;
  msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 20;
  msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.data.resize(scan_pattern_.horizontal_samples.size() *
                  scan_pattern_.horizontal_samples.size() *
                  POINT_STEP);

  uint8_t *ptr = msg.data.data();
  for (size_t i = 0; i < scan_pattern_.horizontal_samples.size(); i++) {
    for (size_t j = 0; j < scan_pattern_.vertical_samples.size(); j++) {
      int ray_index = scan_pattern_.horizontal_samples[i].index +
        scan_pattern_.vertical_samples[j].index * scan_pattern_.horizontal_samples.size();

      // Range
      double r = _msg->scan().ranges(ray_index);
      // Intensity
      double intensity = _msg->scan().intensities(ray_index);
      // Ignore points that lay outside range bands or optionally, beneath a
      // minimum intensity level.
      if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY) ) {
        continue;
      }

      // Noise
      if (gaussian_noise_ != 0.0) {
        r += gaussianKernel(0,gaussian_noise_);
      }

      // Get angles of ray to get xyz for point
      double yAngle = scan_pattern_.horizontal_samples[i].angle.Radian();
      double pAngle = scan_pattern_.vertical_samples[j].angle.Radian();

      // pAngle is rotated by yAngle:
      if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
        *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle);
        *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle);
#if GAZEBO_MAJOR_VERSION > 2
        *((float*)(ptr + 8)) = r * sin(pAngle);
#else
        *((float*)(ptr + 8)) = -r * sin(pAngle);
#endif
        *((float*)(ptr + 16)) = intensity;
#if GAZEBO_MAJOR_VERSION > 2
        *((uint16_t*)(ptr + 20)) = j; // ring
#else
        *((uint16_t*)(ptr + 20)) = scan_pattern_.vertical_samples.size() - 1 - j; // ring
#endif
        ptr += POINT_STEP;
      }
    }
  }

  // Populate message with number of valid points
  msg.point_step = POINT_STEP;
  msg.row_step = ptr - msg.data.data();
  msg.height = 1;
  msg.width = msg.row_step / POINT_STEP;
  msg.is_bigendian = false;
  msg.is_dense = true;
  msg.data.resize(msg.row_step); // Shrink to actual size

  // Publish output
  pub_->publish(msg);
}

} // namespace gazebo
