#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using Point3 = Eigen::Vector3d;
using Rotation3 = Eigen::Matrix3d;
using Clock = std::chrono::steady_clock;

double seconds_between(const Clock::time_point &start,
                       const Clock::time_point &end) {
  return std::chrono::duration<double>(end - start).count();
}

double median(std::vector<double> values) {
  if (values.empty()) {
    throw std::runtime_error("median of empty vector");
  }
  const auto middle =
      values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  if (values.size() % 2U != 0U) {
    return *middle;
  }
  const auto lower = std::max_element(values.begin(), middle);
  return 0.5 * (*lower + *middle);
}

std::string json_ints(const std::vector<int> &values) {
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << values[index];
  }
  output << ']';
  return output.str();
}

std::string json_strings(const std::vector<std::string> &values) {
  std::ostringstream output;
  output << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      output << ',';
    }
    output << '"' << values[index] << '"';
  }
  output << ']';
  return output.str();
}

class PositionKalman3D {
public:
  PositionKalman3D(double lateral_std_m, double depth_std_m,
                   double acceleration_std_m_s2, double reset_gap_s)
      : acceleration_variance_(acceleration_std_m_s2 * acceleration_std_m_s2),
        reset_gap_s_(reset_gap_s) {
    measurement_covariance_.setZero();
    measurement_covariance_.diagonal() << lateral_std_m * lateral_std_m,
        lateral_std_m * lateral_std_m, depth_std_m * depth_std_m;
  }

  Point3 update(const Point3 &measurement, double time_s) {
    if (!measurement.allFinite() || !std::isfinite(time_s)) {
      throw std::runtime_error("non-finite Kalman input");
    }
    if (!initialized_ || time_s <= time_s_ || time_s - time_s_ > reset_gap_s_) {
      state_.setZero();
      state_.head<3>() = measurement;
      covariance_.setZero();
      covariance_.diagonal() << measurement_covariance_(0, 0),
          measurement_covariance_(1, 1), measurement_covariance_(2, 2), 0.1,
          0.1, 0.1;
      time_s_ = time_s;
      initialized_ = true;
      return measurement;
    }

    const double dt = time_s - time_s_;
    Eigen::Matrix<double, 6, 6> transition =
        Eigen::Matrix<double, 6, 6>::Identity();
    transition.block<3, 3>(0, 3) = Rotation3::Identity() * dt;
    Eigen::Matrix<double, 6, 6> process = Eigen::Matrix<double, 6, 6>::Zero();
    const double a = 0.5 * dt * dt;
    process.block<3, 3>(0, 0) =
        Rotation3::Identity() * a * a * acceleration_variance_;
    process.block<3, 3>(0, 3) =
        Rotation3::Identity() * a * dt * acceleration_variance_;
    process.block<3, 3>(3, 0) = process.block<3, 3>(0, 3);
    process.block<3, 3>(3, 3) =
        Rotation3::Identity() * dt * dt * acceleration_variance_;

    const auto predicted_state = transition * state_;
    const auto predicted_covariance =
        transition * covariance_ * transition.transpose() + process;
    const Eigen::Matrix3d innovation_covariance =
        predicted_covariance.block<3, 3>(0, 0) + measurement_covariance_;
    const Eigen::Matrix<double, 6, 3> gain =
        predicted_covariance.block<6, 3>(0, 0) *
        innovation_covariance.inverse();
    state_ = predicted_state + gain * (measurement - predicted_state.head<3>());
    Eigen::Matrix<double, 3, 6> observation =
        Eigen::Matrix<double, 3, 6>::Zero();
    observation.block<3, 3>(0, 0) = Rotation3::Identity();
    const auto correction =
        Eigen::Matrix<double, 6, 6>::Identity() - gain * observation;
    covariance_ = correction * predicted_covariance * correction.transpose() +
                  gain * measurement_covariance_ * gain.transpose();
    time_s_ = time_s;
    return state_.head<3>();
  }

private:
  Eigen::Matrix3d measurement_covariance_ = Eigen::Matrix3d::Zero();
  double acceleration_variance_;
  double reset_gap_s_;
  bool initialized_ = false;
  double time_s_ = 0.0;
  Eigen::Matrix<double, 6, 1> state_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> covariance_ = Eigen::Matrix<double, 6, 6>::Zero();
};

struct AcquiredFrame {
  AcquiredFrame(rs2::frameset frames_value, rs2::video_frame colour_value,
                rs2::depth_frame depth_value)
      : frames(std::move(frames_value)), colour_frame(std::move(colour_value)),
        depth_frame(std::move(depth_value)) {}

  rs2::frameset frames;
  rs2::video_frame colour_frame;
  rs2::depth_frame depth_frame;
  cv::Mat colour;
  cv::Mat depth;
  rs2_intrinsics intrinsics{};
};

class RealSenseSource {
public:
  RealSenseSource(const std::string &requested_serial, int width, int height,
                  int fps)
      : align_(RS2_STREAM_COLOR) {
    rs2::config configuration;
    if (!requested_serial.empty()) {
      configuration.enable_device(requested_serial);
    }
    configuration.enable_stream(RS2_STREAM_COLOR, width, height,
                                RS2_FORMAT_BGR8, fps);
    configuration.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,
                                fps);
    const auto profile = pipeline_.start(configuration);
    const auto device = profile.get_device();
    serial_ = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    depth_scale_ = device.first<rs2::depth_sensor>().get_depth_scale();
  }

  ~RealSenseSource() {
    try {
      pipeline_.stop();
    } catch (const rs2::error &) {
    }
  }

  std::optional<AcquiredFrame> poll() {
    rs2::frameset incoming;
    if (!pipeline_.poll_for_frames(&incoming)) {
      return std::nullopt;
    }
    auto aligned = align_.process(incoming).as<rs2::frameset>();
    auto colour = aligned.get_color_frame();
    auto depth = aligned.get_depth_frame();
    if (!colour || !depth) {
      return std::nullopt;
    }
    AcquiredFrame output(std::move(aligned), std::move(colour),
                         std::move(depth));
    output.colour = cv::Mat(
        cv::Size(output.colour_frame.get_width(),
                 output.colour_frame.get_height()),
        CV_8UC3, const_cast<void *>(output.colour_frame.get_data()),
        static_cast<std::size_t>(output.colour_frame.get_stride_in_bytes()));
    output.depth = cv::Mat(
        cv::Size(output.depth_frame.get_width(),
                 output.depth_frame.get_height()),
        CV_16UC1, const_cast<void *>(output.depth_frame.get_data()),
        static_cast<std::size_t>(output.depth_frame.get_stride_in_bytes()));
    output.intrinsics = output.depth_frame.get_profile()
                            .as<rs2::video_stream_profile>()
                            .get_intrinsics();
    return output;
  }

  const std::string &serial() const { return serial_; }
  float depth_scale() const { return depth_scale_; }

private:
  rs2::pipeline pipeline_;
  rs2::align align_;
  std::string serial_;
  float depth_scale_ = 0.001F;
};

struct DepthResult {
  double depth_m;
  int samples;
};

std::optional<cv::Point2f>
diagonal_center(const std::vector<cv::Point2f> &corners) {
  const auto homogeneous = [](const cv::Point2f &point) {
    return Eigen::Vector3d(point.x, point.y, 1.0);
  };
  const Eigen::Vector3d first =
      homogeneous(corners[0]).cross(homogeneous(corners[2]));
  const Eigen::Vector3d second =
      homogeneous(corners[1]).cross(homogeneous(corners[3]));
  const Eigen::Vector3d intersection = first.cross(second);
  if (!intersection.allFinite() || std::abs(intersection.z()) < 1e-9) {
    return std::nullopt;
  }
  return cv::Point2f(static_cast<float>(intersection.x() / intersection.z()),
                     static_cast<float>(intersection.y() / intersection.z()));
}

std::optional<DepthResult>
robust_marker_depth(const cv::Mat &depth, float depth_scale,
                    const std::vector<cv::Point2f> &corners,
                    double inner_fraction, int minimum_samples,
                    double minimum_depth_m, double maximum_depth_m) {
  const auto center = diagonal_center(corners);
  if (!center) {
    return std::nullopt;
  }
  std::vector<cv::Point> polygon;
  polygon.reserve(4);
  for (const auto &corner : corners) {
    polygon.emplace_back(
        cvRound(center->x + inner_fraction * (corner.x - center->x)),
        cvRound(center->y + inner_fraction * (corner.y - center->y)));
  }
  cv::Rect bounds =
      cv::boundingRect(polygon) & cv::Rect(0, 0, depth.cols, depth.rows);
  if (bounds.empty()) {
    return std::nullopt;
  }
  cv::Mat mask(bounds.height, bounds.width, CV_8UC1, cv::Scalar(0));
  for (auto &point : polygon) {
    point -= bounds.tl();
  }
  cv::fillConvexPoly(mask, polygon, cv::Scalar(1));
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(bounds.area()));
  for (int y = 0; y < bounds.height; ++y) {
    const auto *mask_row = mask.ptr<std::uint8_t>(y);
    const auto *depth_row = depth.ptr<std::uint16_t>(bounds.y + y);
    for (int x = 0; x < bounds.width; ++x) {
      if (mask_row[x] == 0U) {
        continue;
      }
      const double value =
          depth_row[bounds.x + x] * static_cast<double>(depth_scale);
      if (std::isfinite(value) && value >= minimum_depth_m &&
          value <= maximum_depth_m) {
        values.push_back(value);
      }
    }
  }
  if (static_cast<int>(values.size()) < minimum_samples) {
    return std::nullopt;
  }
  const double centre_depth = median(values);
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (const double value : values) {
    deviations.push_back(std::abs(value - centre_depth));
  }
  const double mad = median(deviations);
  if (mad > 0.0) {
    const double limit = 3.5 * 1.4826 * mad;
    values.erase(std::remove_if(values.begin(), values.end(),
                                [centre_depth, limit](double value) {
                                  return std::abs(value - centre_depth) > limit;
                                }),
                 values.end());
  }
  if (static_cast<int>(values.size()) < minimum_samples) {
    return std::nullopt;
  }
  return DepthResult{median(values), static_cast<int>(values.size())};
}

Rotation3 robot_rotation_from_bases(const Point3 &left, const Point3 &right,
                                    double minimum_baseline_m) {
  const Point3 baseline = right - left;
  const double length = baseline.norm();
  if (!std::isfinite(length) || length < minimum_baseline_m) {
    throw std::runtime_error("base-marker baseline is too small");
  }
  const Point3 x_axis = baseline / length;
  const Point3 optical_down(0.0, 1.0, 0.0);
  Point3 y_axis = optical_down - optical_down.dot(x_axis) * x_axis;
  if (y_axis.norm() < 1e-6) {
    throw std::runtime_error(
        "base baseline is degenerate with camera vertical");
  }
  y_axis.normalize();
  Point3 z_axis = x_axis.cross(y_axis).normalized();
  Rotation3 rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  return rotation;
}

Rotation3 blend_rotation(const Rotation3 &previous, const Rotation3 &current,
                         double alpha) {
  const Rotation3 blended = (1.0 - alpha) * previous + alpha * current;
  Eigen::JacobiSVD<Rotation3> decomposition(blended, Eigen::ComputeFullU |
                                                         Eigen::ComputeFullV);
  Rotation3 left = decomposition.matrixU();
  const Rotation3 right = decomposition.matrixV();
  Rotation3 result = left * right.transpose();
  if (result.determinant() < 0.0) {
    left.col(2) *= -1.0;
    result = left * right.transpose();
  }
  return result;
}

} // namespace

class ArucoShapeTrackerCpp : public rclcpp::Node {
public:
  ArucoShapeTrackerCpp() : Node("aruco_shape_tracker") {
    declare_parameters();
    read_parameters();
    configure_arms();
    configure_detector_and_filters();
    source_ = std::make_unique<RealSenseSource>(camera_serial_, width_, height_,
                                                fps_);
    camera_serial_ = source_->serial();
    image_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "aruco/annotated/compressed", rclcpp::SensorDataQoS());
    status_publisher_ =
        create_publisher<std_msgs::msg::String>("aruco/status", 10);
    reset_reference_service_ = create_service<std_srvs::srv::Trigger>(
        "aruco/reset_reference",
        std::bind(&ArucoShapeTrackerCpp::reset_reference, this,
                  std::placeholders::_1, std::placeholders::_2));
    timing_started_ = Clock::now();
    const auto period = std::chrono::nanoseconds(1000000000LL / fps_);
    timer_ = create_wall_timer(
        period, std::bind(&ArucoShapeTrackerCpp::process_frame, this));
    RCLCPP_INFO(get_logger(),
                "C++ RGB-D ArUco tracker active on RealSense %s at %dx%d@%d",
                camera_serial_.c_str(), width_, height_, fps_);
  }

private:
  struct Arm {
    std::string name;
    int reference_id;
    std::array<int, 3> moving_ids;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr state_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr raw_publisher;
  };

  void
  reset_reference(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    base_rotation_.reset();
    reference_origins_.clear();
    for (auto &[id, values] : reference_samples_) {
      (void)id;
      values.clear();
    }
    response->success = true;
    response->message =
        "reference samples cleared; keep both base markers visible";
    RCLCPP_INFO(get_logger(), "reference frame reset requested");
  }

  void declare_parameters() {
    declare_parameter("camera_backend", "realsense_rgbd");
    declare_parameter("camera_serial", "");
    declare_parameter("width", 1280);
    declare_parameter("height", 720);
    declare_parameter("fps", 30);
    declare_parameter("dictionary", "DICT_4X4_50");
    declare_parameter("adaptive_threshold_window_max", 23);
    declare_parameter("fallback_threshold_window_size", 53);
    declare_parameter("minimum_marker_perimeter_rate", 0.03);
    declare_parameter("polygonal_approx_accuracy_rate", 0.03);
    declare_parameter("error_correction_rate", 0.80);
    declare_parameter("minimum_reference_area_px2", 200.0);
    declare_parameter("minimum_base_baseline_m", 0.10);
    declare_parameter("depth_inner_fraction", 0.60);
    declare_parameter("minimum_depth_samples", 10);
    declare_parameter("minimum_depth_m", 0.10);
    declare_parameter("maximum_depth_m", 5.0);
    declare_parameter("enable_kalman", true);
    declare_parameter("kalman_lateral_std_m", 0.0006);
    declare_parameter("kalman_depth_std_m", 0.0015);
    declare_parameter("kalman_acceleration_std_m_s2", 0.50);
    declare_parameter("kalman_reset_gap_s", 0.50);
    declare_parameter("base_rotation_filter_alpha", 0.15);
    declare_parameter("freeze_reference_markers", true);
    declare_parameter("reference_lock_frames", 30);
    declare_parameter("publish_annotated_image", true);
    declare_parameter("arm_names", std::vector<std::string>{"arm_1"});
    declare_parameter("reference_marker_ids", std::vector<std::int64_t>{0});
    declare_parameter("moving_marker_ids", std::vector<std::int64_t>{2, 3, 4});
    declare_parameter("output_topics",
                      std::vector<std::string>{"/arm_1/aruco_state"});
    declare_parameter("raw_3d_topics",
                      std::vector<std::string>{"/arm_1/aruco_points_3d_raw"});
  }

  void read_parameters() {
    if (get_parameter("camera_backend").as_string() != "realsense_rgbd") {
      throw std::runtime_error(
          "only camera_backend=realsense_rgbd is supported");
    }
    camera_serial_ = get_parameter("camera_serial").as_string();
    width_ = static_cast<int>(get_parameter("width").as_int());
    height_ = static_cast<int>(get_parameter("height").as_int());
    fps_ = static_cast<int>(get_parameter("fps").as_int());
    dictionary_name_ = get_parameter("dictionary").as_string();
    adaptive_threshold_window_max_ = static_cast<int>(
        get_parameter("adaptive_threshold_window_max").as_int());
    fallback_threshold_window_size_ = static_cast<int>(
        get_parameter("fallback_threshold_window_size").as_int());
    minimum_marker_perimeter_rate_ =
        get_parameter("minimum_marker_perimeter_rate").as_double();
    polygonal_approx_accuracy_rate_ =
        get_parameter("polygonal_approx_accuracy_rate").as_double();
    error_correction_rate_ =
        get_parameter("error_correction_rate").as_double();
    minimum_reference_area_ =
        get_parameter("minimum_reference_area_px2").as_double();
    minimum_baseline_ = get_parameter("minimum_base_baseline_m").as_double();
    depth_inner_fraction_ = get_parameter("depth_inner_fraction").as_double();
    minimum_depth_samples_ =
        static_cast<int>(get_parameter("minimum_depth_samples").as_int());
    minimum_depth_ = get_parameter("minimum_depth_m").as_double();
    maximum_depth_ = get_parameter("maximum_depth_m").as_double();
    enable_kalman_ = get_parameter("enable_kalman").as_bool();
    rotation_alpha_ = get_parameter("base_rotation_filter_alpha").as_double();
    freeze_references_ = get_parameter("freeze_reference_markers").as_bool();
    reference_lock_frames_ =
        static_cast<int>(get_parameter("reference_lock_frames").as_int());
    publish_image_ = get_parameter("publish_annotated_image").as_bool();
    if (std::min({width_, height_, fps_, minimum_depth_samples_,
                  reference_lock_frames_}) <= 0) {
      throw std::runtime_error(
          "camera, depth and reference parameters must be positive");
    }
    if (!(rotation_alpha_ > 0.0 && rotation_alpha_ <= 1.0)) {
      throw std::runtime_error("base_rotation_filter_alpha must be in (0, 1]");
    }
    if (adaptive_threshold_window_max_ < 3 ||
        fallback_threshold_window_size_ < 3 ||
        adaptive_threshold_window_max_ % 2 == 0 ||
        fallback_threshold_window_size_ % 2 == 0 ||
        minimum_marker_perimeter_rate_ <= 0.0 ||
        polygonal_approx_accuracy_rate_ <= 0.0 ||
        !(error_correction_rate_ > 0.0 && error_correction_rate_ <= 1.0)) {
      throw std::runtime_error("ArUco detector parameters are outside their valid ranges");
    }
  }

  void configure_arms() {
    const auto names = get_parameter("arm_names").as_string_array();
    const auto references =
        get_parameter("reference_marker_ids").as_integer_array();
    const auto moving = get_parameter("moving_marker_ids").as_integer_array();
    const auto output_topics = get_parameter("output_topics").as_string_array();
    const auto raw_topics = get_parameter("raw_3d_topics").as_string_array();
    if (names.empty() || references.size() != names.size() ||
        output_topics.size() != names.size() ||
        raw_topics.size() != names.size() ||
        moving.size() != 3U * names.size()) {
      throw std::runtime_error("arm configuration lengths are inconsistent");
    }
    for (std::size_t index = 0; index < names.size(); ++index) {
      Arm arm;
      arm.name = names[index];
      arm.reference_id = static_cast<int>(references[index]);
      for (std::size_t offset = 0; offset < 3; ++offset) {
        arm.moving_ids[offset] = static_cast<int>(moving[3U * index + offset]);
        moving_marker_ids_.insert(arm.moving_ids[offset]);
        if (arm.moving_ids[offset] == arm.reference_id) {
          throw std::runtime_error(arm.name +
                                   ": reference ID also appears as moving ID");
        }
      }
      arm.state_publisher = create_publisher<sensor_msgs::msg::PointCloud>(
          output_topics[index], rclcpp::SensorDataQoS());
      arm.raw_publisher = create_publisher<sensor_msgs::msg::PointCloud>(
          raw_topics[index], rclcpp::SensorDataQoS());
      arms_.push_back(std::move(arm));
    }
  }

  void configure_detector_and_filters() {
    static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME>
        dictionaries{
            {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
            {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
            {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
            {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        };
    const auto dictionary = dictionaries.find(dictionary_name_);
    if (dictionary == dictionaries.end()) {
      throw std::runtime_error("unsupported ArUco dictionary: " +
                               dictionary_name_);
    }
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary->second);
    detector_parameters_ = cv::aruco::DetectorParameters::create();
    detector_parameters_->adaptiveThreshWinSizeMax =
        adaptive_threshold_window_max_;
    detector_parameters_->minMarkerPerimeterRate =
        minimum_marker_perimeter_rate_;
    detector_parameters_->polygonalApproxAccuracyRate =
        polygonal_approx_accuracy_rate_;
    detector_parameters_->errorCorrectionRate = error_correction_rate_;
    detector_parameters_->cornerRefinementMethod =
        cv::aruco::CORNER_REFINE_SUBPIX;
    fallback_detector_parameters_ = cv::aruco::DetectorParameters::create();
    fallback_detector_parameters_->adaptiveThreshWinSizeMin =
        fallback_threshold_window_size_;
    fallback_detector_parameters_->adaptiveThreshWinSizeMax =
        fallback_threshold_window_size_;
    fallback_detector_parameters_->adaptiveThreshWinSizeStep = 1;
    fallback_detector_parameters_->minMarkerPerimeterRate =
        minimum_marker_perimeter_rate_;
    fallback_detector_parameters_->polygonalApproxAccuracyRate =
        polygonal_approx_accuracy_rate_;
    fallback_detector_parameters_->errorCorrectionRate = error_correction_rate_;
    fallback_detector_parameters_->cornerRefinementMethod =
        cv::aruco::CORNER_REFINE_SUBPIX;

    const double lateral = get_parameter("kalman_lateral_std_m").as_double();
    const double depth = get_parameter("kalman_depth_std_m").as_double();
    const double acceleration =
        get_parameter("kalman_acceleration_std_m_s2").as_double();
    const double reset = get_parameter("kalman_reset_gap_s").as_double();
    std::set<int> ids;
    for (const auto &arm : arms_) {
      ids.insert(arm.reference_id);
      ids.insert(arm.moving_ids.begin(), arm.moving_ids.end());
      reference_samples_[arm.reference_id] = {};
    }
    for (const int id : ids) {
      filters_.emplace(id,
                       PositionKalman3D(lateral, depth, acceleration, reset));
    }
  }

  static double marker_area(const std::vector<cv::Point2f> &corners) {
    return std::abs(cv::contourArea(corners));
  }

  sensor_msgs::msg::PointCloud
  point_cloud(const rclcpp::Time &stamp, const std::string &frame_id,
              const std::vector<Point3> &points, const std::vector<int> &ids,
              const std::vector<std::pair<std::string, std::vector<float>>>
                  &channels = {}) const {
    sensor_msgs::msg::PointCloud message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;
    for (const auto &point : points) {
      geometry_msgs::msg::Point32 output;
      output.x = static_cast<float>(point.x());
      output.y = static_cast<float>(point.y());
      output.z = static_cast<float>(point.z());
      message.points.push_back(output);
    }
    sensor_msgs::msg::ChannelFloat32 id_channel;
    id_channel.name = "marker_id";
    for (const int id : ids) {
      id_channel.values.push_back(static_cast<float>(id));
    }
    message.channels.push_back(std::move(id_channel));
    for (const auto &[name, values] : channels) {
      sensor_msgs::msg::ChannelFloat32 channel;
      channel.name = name;
      channel.values = values;
      message.channels.push_back(std::move(channel));
    }
    return message;
  }

  bool publish_arm(const Arm &arm,
                   const std::unordered_map<int, Point3> &raw_points,
                   const std::unordered_map<int, Point3> &filtered_points,
                   const std::unordered_map<int, int> &sample_counts,
                   const rclcpp::Time &stamp) {
    const auto origin = reference_origins_.find(arm.reference_id);
    if (origin == reference_origins_.end() || !base_rotation_) {
      return false;
    }
    std::vector<Point3> local;
    std::vector<Point3> raw;
    std::vector<int> ids;
    std::vector<float> counts;
    std::vector<float> raw_z;
    for (const int id : arm.moving_ids) {
      const auto filtered = filtered_points.find(id);
      const auto unfiltered = raw_points.find(id);
      if (filtered == filtered_points.end() || unfiltered == raw_points.end()) {
        return false;
      }
      local.push_back(base_rotation_->transpose() *
                      (filtered->second - origin->second));
      raw.push_back(unfiltered->second);
      ids.push_back(id);
      counts.push_back(static_cast<float>(sample_counts.at(id)));
      raw_z.push_back(static_cast<float>(unfiltered->second.z()));
    }
    arm.state_publisher->publish(point_cloud(
        stamp, "aruco_base_" + std::to_string(arm.reference_id), local, ids,
        {{"depth_samples", counts}, {"raw_camera_z_m", raw_z}}));
    arm.raw_publisher->publish(
        point_cloud(stamp, "camera_" + camera_serial_ + "_optical_frame", raw,
                    ids, {{"depth_samples", counts}}));
    return true;
  }

  void update_reference_frame(
      const std::unordered_map<int, Point3> &filtered_points) {
    std::unordered_map<int, bool> reference_valid;
    for (const auto &arm : arms_) {
      reference_valid[arm.reference_id] =
          filtered_points.count(arm.reference_id) != 0U &&
          current_detections_.count(arm.reference_id) != 0U &&
          marker_area(current_detections_.at(arm.reference_id)) >=
              minimum_reference_area_;
    }
    if (freeze_references_ && !base_rotation_) {
      for (const auto &arm : arms_) {
        if (reference_valid.at(arm.reference_id)) {
          reference_samples_[arm.reference_id].push_back(
              filtered_points.at(arm.reference_id));
          if (reference_samples_[arm.reference_id].size() >
              static_cast<std::size_t>(reference_lock_frames_)) {
            reference_samples_[arm.reference_id].erase(
                reference_samples_[arm.reference_id].begin());
          }
        }
      }
      std::size_t count = reference_samples_.begin()->second.size();
      for (const auto &[id, values] : reference_samples_) {
        (void)id;
        count = std::min(count, values.size());
      }
      if (arms_.size() >= 2U &&
          count >= static_cast<std::size_t>(reference_lock_frames_)) {
        try {
          for (const auto &arm : arms_) {
            Point3 origin;
            for (int axis = 0; axis < 3; ++axis) {
              std::vector<double> values;
              for (const auto &point :
                   reference_samples_.at(arm.reference_id)) {
                values.push_back(point[axis]);
              }
              origin[axis] = median(values);
            }
            reference_origins_[arm.reference_id] = origin;
          }
          base_rotation_ = robot_rotation_from_bases(
              reference_origins_.at(arms_[0].reference_id),
              reference_origins_.at(arms_[1].reference_id), minimum_baseline_);
          RCLCPP_INFO(get_logger(),
                      "reference frame locked from %zu RGB-D samples", count);
        } catch (const std::runtime_error &) {
          for (auto &[id, values] : reference_samples_) {
            (void)id;
            values.clear();
          }
        }
      }
    } else if (!freeze_references_) {
      const bool references_valid =
          std::all_of(reference_valid.begin(), reference_valid.end(),
                      [](const auto &item) { return item.second; });
      if (references_valid && arms_.size() >= 2U) {
        try {
          const Rotation3 measured = robot_rotation_from_bases(
              filtered_points.at(arms_[0].reference_id),
              filtered_points.at(arms_[1].reference_id), minimum_baseline_);
          base_rotation_ =
              base_rotation_
                  ? blend_rotation(*base_rotation_, measured, rotation_alpha_)
                  : measured;
          for (const auto &arm : arms_) {
            reference_origins_[arm.reference_id] =
                filtered_points.at(arm.reference_id);
          }
        } catch (const std::runtime_error &) {
        }
      }
    }
  }

  void publish_annotated(cv::Mat &frame,
                         const std::vector<std::vector<cv::Point2f>> &corners,
                         const std::vector<int> &ids,
                         const std::unordered_map<int, Point3> &raw_points,
                         const std::vector<std::string> &complete,
                         const rclcpp::Time &stamp) {
    if (!publish_image_ || image_publisher_->get_subscription_count() == 0U) {
      return;
    }
    if (!ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, corners, ids);
    }
    for (std::size_t index = 0; index < ids.size(); ++index) {
      const auto point = raw_points.find(ids[index]);
      if (point == raw_points.end()) {
        continue;
      }
      cv::Point2f centre(0.0F, 0.0F);
      for (const auto &corner : corners[index]) {
        centre += corner;
      }
      centre *= 0.25F;
      std::ostringstream label;
      label << std::fixed << std::setprecision(3) << point->second.z() << 'm';
      cv::putText(frame, label.str(), centre, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                  cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }
    std::ostringstream header;
    header << camera_serial_ << " | frame " << frame_index_ << " | ";
    for (std::size_t index = 0; index < complete.size(); ++index) {
      if (index != 0U) {
        header << ',';
      }
      header << complete[index];
    }
    cv::putText(frame, header.str(), cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2,
                cv::LINE_AA);
    std::vector<std::uint8_t> encoded;
    if (cv::imencode(".jpg", frame, encoded, {cv::IMWRITE_JPEG_QUALITY, 85})) {
      sensor_msgs::msg::CompressedImage message;
      message.header.stamp = stamp;
      message.header.frame_id = "camera_" + camera_serial_ + "_optical_frame";
      message.format = "jpeg";
      message.data = std::move(encoded);
      image_publisher_->publish(std::move(message));
    }
  }

  void publish_status(const std::vector<int> &detected,
                      const std::vector<int> &depth_valid,
                      const std::vector<int> &valid_references,
                      const std::vector<std::string> &complete) {
    if (frame_index_ - last_status_frame_ < static_cast<std::uint64_t>(fps_)) {
      return;
    }
    const auto now = Clock::now();
    const double elapsed =
        std::max(seconds_between(timing_started_, now), 1e-9);
    const double count =
        static_cast<double>(std::max<std::uint64_t>(timing_frames_, 1U));
    std::vector<int> cached;
    for (const auto &[id, point] : reference_origins_) {
      (void)point;
      cached.push_back(id);
    }
    std::sort(cached.begin(), cached.end());
    std::size_t reference_count = 0;
    if (!reference_samples_.empty()) {
      reference_count = reference_samples_.begin()->second.size();
      for (const auto &[id, values] : reference_samples_) {
        (void)id;
        reference_count = std::min(reference_count, values.size());
      }
    }
    std::ostringstream json;
    json << std::fixed << std::setprecision(3) << "{\"frame\":" << frame_index_
         << ",\"camera_serial\":\"" << camera_serial_ << "\""
         << ",\"detected_ids\":" << json_ints(detected)
         << ",\"depth_valid_ids\":" << json_ints(depth_valid)
         << ",\"valid_reference_ids\":" << json_ints(valid_references)
         << ",\"base_rotation_ready\":" << (base_rotation_ ? "true" : "false")
         << ",\"reference_frame_locked\":"
         << ((freeze_references_ && base_rotation_) ? "true" : "false")
         << ",\"cached_reference_ids\":" << json_ints(cached)
         << ",\"reference_lock_samples\":" << reference_count
         << ",\"complete_arms\":" << json_strings(complete)
         << ",\"coordinate_frame\":\"metric_robot_base_3d\""
         << ",\"kalman_enabled\":" << (enable_kalman_ ? "true" : "false")
         << ",\"implementation\":\"cpp\""
         << ",\"processed_fps\":" << count / elapsed
         << ",\"acquire_align_ms\":" << 1000.0 * timing_acquire_s_ / count
         << ",\"detect_ms\":" << 1000.0 * timing_detect_s_ / count
         << ",\"measure_publish_ms\":" << 1000.0 * timing_rest_s_ / count
         << ",\"processing_ms\":" << 1000.0 * timing_total_s_ / count
         << ",\"empty_poll_count\":" << empty_reads_ << '}';
    std_msgs::msg::String status;
    status.data = json.str();
    status_publisher_->publish(std::move(status));
    last_status_frame_ = frame_index_;
    timing_started_ = now;
    timing_frames_ = 0;
    timing_acquire_s_ = timing_detect_s_ = timing_rest_s_ = timing_total_s_ =
        0.0;
  }

  void process_frame() {
    const auto total_start = Clock::now();
    auto acquired = source_->poll();
    const auto acquired_at = Clock::now();
    if (!acquired) {
      ++empty_reads_;
      return;
    }
    ++frame_index_;
    const rclcpp::Time stamp = now();
    cv::Mat grey;
    cv::cvtColor(acquired->colour, grey, cv::COLOR_BGR2GRAY);
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(grey, dictionary_, corners, ids,
                             detector_parameters_);
    const bool moving_marker_missing = std::any_of(
        moving_marker_ids_.begin(), moving_marker_ids_.end(),
        [&ids](int expected) {
          return std::find(ids.begin(), ids.end(), expected) == ids.end();
        });
    if (moving_marker_missing) {
      std::vector<std::vector<cv::Point2f>> fallback_corners;
      std::vector<int> fallback_ids;
      cv::aruco::detectMarkers(grey, dictionary_, fallback_corners,
                               fallback_ids, fallback_detector_parameters_);
      for (std::size_t index = 0; index < fallback_ids.size(); ++index) {
        if (std::find(ids.begin(), ids.end(), fallback_ids[index]) ==
            ids.end()) {
          ids.push_back(fallback_ids[index]);
          corners.push_back(std::move(fallback_corners[index]));
        }
      }
    }
    const auto detected_at = Clock::now();

    current_detections_.clear();
    for (std::size_t index = 0; index < ids.size(); ++index) {
      const auto existing = current_detections_.find(ids[index]);
      if (existing == current_detections_.end() ||
          marker_area(corners[index]) > marker_area(existing->second)) {
        current_detections_[ids[index]] = corners[index];
      }
    }

    std::unordered_map<int, Point3> raw_points;
    std::unordered_map<int, Point3> filtered_points;
    std::unordered_map<int, int> sample_counts;
    const double time_s = stamp.seconds();
    for (const auto &[id, marker_corners] : current_detections_) {
      const auto depth = robust_marker_depth(
          acquired->depth, source_->depth_scale(), marker_corners,
          depth_inner_fraction_, minimum_depth_samples_, minimum_depth_,
          maximum_depth_);
      if (!depth) {
        continue;
      }
      cv::Point2f center(0.0F, 0.0F);
      for (const auto &corner : marker_corners) {
        center += corner;
      }
      center *= 0.25F;
      const float pixel[2] = {center.x, center.y};
      float output[3];
      rs2_deproject_pixel_to_point(output, &acquired->intrinsics, pixel,
                                   depth->depth_m);
      const Point3 point(output[0], output[1], output[2]);
      raw_points[id] = point;
      sample_counts[id] = depth->samples;
      const auto filter = filters_.find(id);
      filtered_points[id] = enable_kalman_ && filter != filters_.end()
                                ? filter->second.update(point, time_s)
                                : point;
    }

    update_reference_frame(filtered_points);
    std::vector<std::string> complete;
    if (base_rotation_) {
      for (const auto &arm : arms_) {
        if (publish_arm(arm, raw_points, filtered_points, sample_counts,
                        stamp)) {
          complete.push_back(arm.name);
        }
      }
    }
    publish_annotated(acquired->colour, corners, ids, raw_points, complete,
                      stamp);

    std::vector<int> detected;
    std::vector<int> depth_valid;
    std::vector<int> valid_references;
    for (const auto &[id, marker_corners] : current_detections_) {
      detected.push_back(id);
      if (raw_points.count(id) != 0U) {
        depth_valid.push_back(id);
      }
    }
    for (const auto &arm : arms_) {
      const auto detection = current_detections_.find(arm.reference_id);
      if (detection != current_detections_.end() &&
          filtered_points.count(arm.reference_id) != 0U &&
          marker_area(detection->second) >= minimum_reference_area_) {
        valid_references.push_back(arm.reference_id);
      }
    }
    std::sort(detected.begin(), detected.end());
    std::sort(depth_valid.begin(), depth_valid.end());
    std::sort(valid_references.begin(), valid_references.end());
    const auto finished_at = Clock::now();
    ++timing_frames_;
    timing_acquire_s_ += seconds_between(total_start, acquired_at);
    timing_detect_s_ += seconds_between(acquired_at, detected_at);
    timing_rest_s_ += seconds_between(detected_at, finished_at);
    timing_total_s_ += seconds_between(total_start, finished_at);
    publish_status(detected, depth_valid, valid_references, complete);
  }

  std::string camera_serial_;
  std::string dictionary_name_;
  int width_ = 1280;
  int height_ = 720;
  int fps_ = 30;
  int adaptive_threshold_window_max_ = 23;
  int fallback_threshold_window_size_ = 53;
  double minimum_marker_perimeter_rate_ = 0.03;
  double polygonal_approx_accuracy_rate_ = 0.03;
  double error_correction_rate_ = 0.80;
  double minimum_reference_area_ = 80.0;
  double minimum_baseline_ = 0.10;
  double depth_inner_fraction_ = 0.60;
  int minimum_depth_samples_ = 10;
  double minimum_depth_ = 0.10;
  double maximum_depth_ = 5.0;
  bool enable_kalman_ = true;
  double rotation_alpha_ = 0.15;
  bool freeze_references_ = true;
  int reference_lock_frames_ = 30;
  bool publish_image_ = true;

  std::vector<Arm> arms_;
  std::unique_ptr<RealSenseSource> source_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  cv::Ptr<cv::aruco::DetectorParameters> fallback_detector_parameters_;
  std::set<int> moving_marker_ids_;
  std::unordered_map<int, PositionKalman3D> filters_;
  std::unordered_map<int, std::vector<Point3>> reference_samples_;
  std::unordered_map<int, Point3> reference_origins_;
  std::optional<Rotation3> base_rotation_;
  std::unordered_map<int, std::vector<cv::Point2f>> current_detections_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      image_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_reference_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t frame_index_ = 0;
  std::uint64_t last_status_frame_ = 0;
  std::uint64_t empty_reads_ = 0;
  Clock::time_point timing_started_;
  std::uint64_t timing_frames_ = 0;
  double timing_acquire_s_ = 0.0;
  double timing_detect_s_ = 0.0;
  double timing_rest_s_ = 0.0;
  double timing_total_s_ = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ArucoShapeTrackerCpp>());
  } catch (const std::exception &error) {
    std::cerr << "aruco_shape_tracker_cpp: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
