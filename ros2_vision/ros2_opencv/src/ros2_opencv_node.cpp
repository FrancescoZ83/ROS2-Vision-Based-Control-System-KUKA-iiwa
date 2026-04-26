#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
  
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
        
    timer_ = this->create_wall_timer(
        30ms, std::bind(&MinimalImagePublisher::timer_callback, this));
        
    sim_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/videocamera", 10, std::bind(&MinimalImagePublisher::img_subscriber, this, std::placeholders::_1));
  }
 
private:
  void timer_callback() {
    // Write message to be sent. Member function toImageMsg() converts a CvImage
    // into a ROS image message
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_pub)
               .toImageMsg();
 
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    //RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
    count_++;
  }
  
  void img_subscriber(const sensor_msgs::msg::Image::SharedPtr sensor_msg) {
    // Convert ROS image to OpenCV image
    cv::Mat cv_image = cv_bridge::toCvCopy(sensor_msg, "bgr8")->image;

    // Convert the image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the range for blue color in HSV
    cv::Scalar lower_blue(100, 150, 50); // Adjust these values for your lighting conditions
    cv::Scalar upper_blue(140, 255, 255);

    // Create a mask for blue regions
    cv::Mat mask;
    cv::inRange(hsv_image, lower_blue, upper_blue, mask);

    // Configure blob detector parameters
    cv::SimpleBlobDetector::Params params;
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByArea = true;
    params.minArea = 0;
    params.maxArea = 500000;
    
    // Create a blob detector
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;

    // Detect blobs in the mask
    detector->detect(mask, keypoints);

    // Draw detected blobs as red circles
    cv::Mat im_with_keypoints;
    cv::drawKeypoints(cv_image, keypoints, im_with_keypoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
    // Showing Images
    cv::imshow("keypoints", im_with_keypoints );
    cv::waitKey(1);
    
    // Image to publish
    img_pub = im_with_keypoints;
  }


  
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sim_image_subscriber_;
  
  cv::Mat img_pub;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
