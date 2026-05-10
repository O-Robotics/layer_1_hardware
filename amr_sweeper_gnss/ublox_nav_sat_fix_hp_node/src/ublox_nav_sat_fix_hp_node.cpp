// Copyright 2023 CMP Engineers Pty Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "ublox_nav_sat_fix_hp_node/visibility_control.h"
#include "amr_sweeper_gnss/msg/gps_fix.hpp"
#include "amr_sweeper_gnss/msg/ubx_nav_cov.hpp"
#include "amr_sweeper_gnss/msg/ubx_nav_hp_pos_llh.hpp"
#include "amr_sweeper_gnss/msg/ubx_nav_status.hpp"

using std::placeholders::_1;

// size of position covariance array
static const size_t POS_COV_ARR_SIZE = 9;

namespace ublox_nav_sat_fix_hp
{

class UbloxNavSatHpFixNode : public rclcpp::Node
{
public:
  UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC
  explicit UbloxNavSatHpFixNode(const rclcpp::NodeOptions & options)
  : Node("ublox_nav_sat_fix_hp",
      rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    enu_pos_cov_.fill(0.0);  // initialise values to zero

    if (!has_parameter("min_fix_type")) {
      declare_parameter(
        "min_fix_type", static_cast<int>(amr_sweeper_gnss::msg::GpsFix::GPS_FIX_3D));
    }
    if (!has_parameter("min_horizontal_stddev_m")) {
      declare_parameter("min_horizontal_stddev_m", 1.5);
    }
    if (!has_parameter("min_vertical_stddev_m")) {
      declare_parameter("min_vertical_stddev_m", 3.0);
    }
    if (!has_parameter("horizontal_covariance_scale")) {
      declare_parameter("horizontal_covariance_scale", 4.0);
    }
    if (!has_parameter("vertical_covariance_scale")) {
      declare_parameter("vertical_covariance_scale", 4.0);
    }
    if (!has_parameter("use_hacc_vacc_covariance_floor")) {
      declare_parameter("use_hacc_vacc_covariance_floor", true);
    }

    min_fix_type_ = get_parameter("min_fix_type").as_int();
    min_horizontal_stddev_m_ = get_parameter("min_horizontal_stddev_m").as_double();
    min_vertical_stddev_m_ = get_parameter("min_vertical_stddev_m").as_double();
    horizontal_covariance_scale_ = get_parameter("horizontal_covariance_scale").as_double();
    vertical_covariance_scale_ = get_parameter("vertical_covariance_scale").as_double();
    use_hacc_vacc_covariance_floor_ = get_parameter("use_hacc_vacc_covariance_floor").as_bool();

    auto qos = rclcpp::SensorDataQoS();

    // Create publishers
    nav_sat_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", qos);

    // Create subscribers
    ubx_nav_hp_pos_llh_sub_ = this->create_subscription<amr_sweeper_gnss::msg::UBXNavHPPosLLH>(
      "ubx_nav_hp_pos_llh", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_hp_pos_llh_callback, this, std::placeholders::_1));
    ubx_nav_cov_sub_ = this->create_subscription<amr_sweeper_gnss::msg::UBXNavCov>(
      "ubx_nav_cov", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_cov_callback, this, std::placeholders::_1));
    ubx_nav_status_sub_ = this->create_subscription<amr_sweeper_gnss::msg::UBXNavStatus>(
      "ubx_nav_status", qos,
      std::bind(&UbloxNavSatHpFixNode::nav_sta_callback, this, std::placeholders::_1));
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  ~UbloxNavSatHpFixNode() {RCLCPP_INFO(this->get_logger(), "finished");}

private:
  // void nav_hp_pos_llh_callback(const amr_sweeper_gnss::msg::UBXNavHPPosLLH::SharedPtr llh_msg);
  // void nav_cov_callback(const amr_sweeper_gnss::msg::UBXNavCov::SharedPtr nav_cov_msg);
  // void nav_sta_callback(const amr_sweeper_gnss::msg::UBXNavStatus::SharedPtr nav_sta_msg);

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_pub_;

  rclcpp::Subscription<amr_sweeper_gnss::msg::UBXNavHPPosLLH>::SharedPtr ubx_nav_hp_pos_llh_sub_;
  rclcpp::Subscription<amr_sweeper_gnss::msg::UBXNavCov>::SharedPtr ubx_nav_cov_sub_;
  rclcpp::Subscription<amr_sweeper_gnss::msg::UBXNavStatus>::SharedPtr ubx_nav_status_sub_;

  // std::vector<double> enu_covariance_diagonal_;
  std::array<double, POS_COV_ARR_SIZE> enu_pos_cov_;
  sensor_msgs::msg::NavSatStatus nav_sat_stat_;
  int min_fix_type_;
  double min_horizontal_stddev_m_;
  double min_vertical_stddev_m_;
  double horizontal_covariance_scale_;
  double vertical_covariance_scale_;
  bool use_hacc_vacc_covariance_floor_;

  // flags used to check whether we have received corresponding messages
  bool have_recd_enu_pos_cov_ = false;
  int last_fix_type_ = amr_sweeper_gnss::msg::GpsFix::GPS_NO_FIX;

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_hp_pos_llh_callback(
    const amr_sweeper_gnss::msg::UBXNavHPPosLLH::SharedPtr ubx_hppos_llh_msg)
  {
    if (ubx_hppos_llh_msg->invalid_lon || ubx_hppos_llh_msg->invalid_lat ||
      ubx_hppos_llh_msg->invalid_height)
    {
      RCLCPP_DEBUG(this->get_logger(), "Skipping NavSatFix publish because HPPOSLLH is invalid");
      return;
    }

    if (last_fix_type_ < min_fix_type_) {
      RCLCPP_DEBUG(
        this->get_logger(), "Skipping NavSatFix publish because fix type %d is below minimum %d",
        last_fix_type_, min_fix_type_);
      return;
    }

    // Create the NavSatFix message
    sensor_msgs::msg::NavSatFix nav_sat_fix_msg;
    // header - copy from Pos message
    nav_sat_fix_msg.header = ubx_hppos_llh_msg->header;
    // copy status from previous nav_sat_stat message
    nav_sat_fix_msg.status = nav_sat_stat_;

    // Extract the LLH and high-precision components
    double lat = ubx_hppos_llh_msg->lat * 1e-7 + ubx_hppos_llh_msg->lat_hp * 1e-9;
    double lon = ubx_hppos_llh_msg->lon * 1e-7 + ubx_hppos_llh_msg->lon_hp * 1e-9;
    double alt = ubx_hppos_llh_msg->height * 1e-3 + ubx_hppos_llh_msg->height_hp * 1e-4;

    // Convert the LLH position and covariance values to NavSatFix message format
    nav_sat_fix_msg.latitude = lat;   // Degrees
    nav_sat_fix_msg.longitude = lon;  // Degrees
    nav_sat_fix_msg.altitude = alt;   // meters

    // Fill in covariance data
    if (nav_sat_fix_msg.position_covariance.size() != enu_pos_cov_.size()) {
      RCLCPP_ERROR(
        this->get_logger(), "Size mismatch betwwen NavSatFix covariance data and EnuPosCov data");
      return;
    }
    for (size_t i = 0; i < enu_pos_cov_.size(); i++) {
      nav_sat_fix_msg.position_covariance[i] = enu_pos_cov_[i];
    }
    apply_covariance_floor(*ubx_hppos_llh_msg, nav_sat_fix_msg);
    if (have_recd_enu_pos_cov_) {
      // Set covariance type to estimated from the converted NED to ENU covariance
      nav_sat_fix_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    } else {
      nav_sat_fix_msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    }

    // Publish NavSatFix message
    nav_sat_fix_pub_->publish(nav_sat_fix_msg);

    RCLCPP_DEBUG(
      this->get_logger(), "Published NavSatFix with lat %4f lon %4f alt %4f", lat, lon, alt);
  }

  void apply_covariance_floor(
    const amr_sweeper_gnss::msg::UBXNavHPPosLLH & ubx_hppos_llh_msg,
    sensor_msgs::msg::NavSatFix & nav_sat_fix_msg)
  {
    const double min_horizontal_var = min_horizontal_stddev_m_ * min_horizontal_stddev_m_;
    const double min_vertical_var = min_vertical_stddev_m_ * min_vertical_stddev_m_;

    double hacc_var = min_horizontal_var;
    double vacc_var = min_vertical_var;
    if (use_hacc_vacc_covariance_floor_) {
      const double hacc_m = static_cast<double>(ubx_hppos_llh_msg.h_acc) * 1e-4;
      const double vacc_m = static_cast<double>(ubx_hppos_llh_msg.v_acc) * 1e-4;
      hacc_var = std::pow(hacc_m * horizontal_covariance_scale_, 2);
      vacc_var = std::pow(vacc_m * vertical_covariance_scale_, 2);
    }

    nav_sat_fix_msg.position_covariance[0] =
      std::max(nav_sat_fix_msg.position_covariance[0], std::max(min_horizontal_var, hacc_var));
    nav_sat_fix_msg.position_covariance[4] =
      std::max(nav_sat_fix_msg.position_covariance[4], std::max(min_horizontal_var, hacc_var));
    nav_sat_fix_msg.position_covariance[8] =
      std::max(nav_sat_fix_msg.position_covariance[8], std::max(min_vertical_var, vacc_var));
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_cov_callback(const amr_sweeper_gnss::msg::UBXNavCov::SharedPtr ubx_cov_msg)
  {
    if (!ubx_cov_msg->pos_cor_valid) {
      have_recd_enu_pos_cov_ = false;
      return;
    }

    // 6 position covariance values available in UBX-NAV-COV matrix
    // Matrix is symmetrix, so only upper triangular values are shown
    // pos_cov_nn
    // pos_cov_ne
    // pos_cov_nd
    // pos_cov_ee
    // pos_cov_ed
    // pos_cov_dd

    // In matrix notation, the values in NED coordinate system are
    // C_NED = | Pnn Pne Pnd |
    //         | Pne Pee Ped |
    //         | Pnd Ped Pdd |

    // After transformation into ENU coordinate system, the matrix becomes
    // C_ENU = | Pee  Pne -Ped |
    //         | Pne  Pnn -Pnd |
    //         |-Ped -Pnd  Pdd |

    // Tranform the covariance matrix from NED to ENU format in row-major order
    static_assert(POS_COV_ARR_SIZE == 9, "size of enu_pos_cov_ must be 9");
    enu_pos_cov_[0] = ubx_cov_msg->pos_cov_ee;
    enu_pos_cov_[1] = ubx_cov_msg->pos_cov_ne;
    enu_pos_cov_[2] = -ubx_cov_msg->pos_cov_ed;
    enu_pos_cov_[3] = ubx_cov_msg->pos_cov_ne;
    enu_pos_cov_[4] = ubx_cov_msg->pos_cov_nn;
    enu_pos_cov_[5] = -ubx_cov_msg->pos_cov_nd;
    enu_pos_cov_[6] = -ubx_cov_msg->pos_cov_ed;
    enu_pos_cov_[7] = -ubx_cov_msg->pos_cov_nd;
    enu_pos_cov_[8] = ubx_cov_msg->pos_cov_dd;

    // set flag to show we have received fresh data for this message
    have_recd_enu_pos_cov_ = true;
  }

  UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
  void nav_sta_callback(const amr_sweeper_gnss::msg::UBXNavStatus::SharedPtr ubx_sta_msg)
  {
    last_fix_type_ = ubx_sta_msg->gps_fix.fix_type;

    // UBX NAV STATUS values do not map very cleanly to ROS2 sensor_msgs/msg/NavSatStatus values.
    // Do the best we can to indicate whether we have GPS fix or not
    switch (ubx_sta_msg->gps_fix.fix_type) {
      case amr_sweeper_gnss::msg::GpsFix::GPS_NO_FIX:
      case amr_sweeper_gnss::msg::GpsFix::GPS_TIME_ONLY:
      case amr_sweeper_gnss::msg::GpsFix::GPS_DEAD_RECKONING_ONLY:
        nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
      case amr_sweeper_gnss::msg::GpsFix::GPS_FIX_2D:
      case amr_sweeper_gnss::msg::GpsFix::GPS_FIX_3D:
      case amr_sweeper_gnss::msg::GpsFix::GPS_PLUS_DEAD_RECKONING:
        if (true == ubx_sta_msg->diff_soln) {  // diff corrections were applied
          nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
          nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        break;
      default:
        nav_sat_stat_.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        break;
    }

    // Service values - derive from UBX-NAV-SAT gnssId field?
    // In their absence, use arrogant default assumption of GPS
    nav_sat_stat_.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }
};

}  // namespace ublox_nav_sat_fix_hp

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode)
