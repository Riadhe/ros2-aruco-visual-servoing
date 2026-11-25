#include <cmath>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "erl1_interfaces/msg/marker_detection.hpp"
#include "erl1_interfaces/msg/marker_detection_array.hpp"

using erl1_interfaces::msg::MarkerDetection;
using erl1_interfaces::msg::MarkerDetectionArray;

class ArucoDetector : public rclcpp::Node
{
public:
  ArucoDetector()
  : Node("aruco_detector")
  {
    // 1. Parameters: every topic name is configurable (same code on the ROSbot)
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image");
    const auto transport = declare_parameter<std::string>("image_transport", "raw");
    const auto detections_topic =
      declare_parameter<std::string>("detections_topic", "/aruco/detections");
    const auto marker_image_topic =
      declare_parameter<std::string>("marker_image_topic", "/marker_image");
    const auto reached_topic = declare_parameter<std::string>("reached_topic", "/hunt/reached");

    // 2. ArUco setup (OpenCV 4.6 API)
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    detector_params_ = cv::aruco::DetectorParameters::create();

    // 3. Publishers
    detections_pub_ = create_publisher<MarkerDetectionArray>(detections_topic, 10);
    marker_image_pub_ = image_transport::create_publisher(this, marker_image_topic);

    // 4. Subscribers
    image_sub_ = image_transport::create_subscription(
      this, image_topic,
      std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1),
      transport);
    reached_sub_ = create_subscription<std_msgs::msg::Int32>(
      reached_topic, 10,
      std::bind(&ArucoDetector::reached_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening on %s (transport: %s)",
      image_topic.c_str(), transport.c_str());
  }

private:
  // Called for every camera frame
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, detector_params_);

    MarkerDetectionArray out;
    out.header = msg->header;
    out.image_width = msg->width;
    out.image_height = msg->height;
    for (size_t i = 0; i < ids.size(); ++i) {
      out.markers.push_back(to_detection(ids[i], corners[i]));
    }
    detections_pub_->publish(out);

    // Keep the last frame: used when node 4 says a marker was reached
    last_image_ = cv_ptr;
    last_detections_ = out;
  }

  // Node 4 publishes the id of the marker the robot just reached
  void reached_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!last_image_) {
      RCLCPP_WARN(get_logger(), "No image received yet");
      return;
    }
    for (const auto & m : last_detections_.markers) {
      if (m.id != msg->data) {
        continue;
      }
      cv::Mat img = last_image_->image.clone();
      const int cx = static_cast<int>(std::round(m.u));
      const int cy = static_cast<int>(std::round(m.v));
      const int r = static_cast<int>(m.side * 0.75);  // circle around the whole square
      cv::circle(img, cv::Point(cx, cy), r, cv::Scalar(0, 255, 0), 3);
      cv::putText(img, "id " + std::to_string(m.id), cv::Point(cx - r, cy - r - 10),
        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
      marker_image_pub_.publish(
        cv_bridge::CvImage(last_image_->header, "bgr8", img).toImageMsg());
      RCLCPP_INFO(get_logger(), "Published /marker_image for marker %d", m.id);
      return;
    }
    RCLCPP_WARN(get_logger(), "Marker %d is not in the last image", msg->data);
  }

  // 4 corners in pixels -> center (u, v) and mean side length
  static MarkerDetection to_detection(int id, const std::vector<cv::Point2f> & c)
  {
    MarkerDetection d;
    d.id = id;
    d.u = (c[0].x + c[1].x + c[2].x + c[3].x) / 4.0f;
    d.v = (c[0].y + c[1].y + c[2].y + c[3].y) / 4.0f;
    float side = 0.0f;
    for (int k = 0; k < 4; ++k) {
      side += static_cast<float>(cv::norm(c[k] - c[(k + 1) % 4]));
    }
    d.side = side / 4.0f;
    return d;
  }

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  rclcpp::Publisher<MarkerDetectionArray>::SharedPtr detections_pub_;
  image_transport::Publisher marker_image_pub_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr reached_sub_;
  cv_bridge::CvImagePtr last_image_;
  MarkerDetectionArray last_detections_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}