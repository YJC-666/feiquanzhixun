#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "camera_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // --- parameters ---
  int device_id;
  int width, height;
  std::string frame_id;
  bool publish_compressed_only;

  pnh.param<int>("device_id", device_id, 0);
  pnh.param<int>("width", width, 640);
  pnh.param<int>("height", height, 480);
  pnh.param<std::string>("frame_id", frame_id, "camera");
  pnh.param<bool>("compressed_only", publish_compressed_only, false);

  // --- open camera ---
  cv::VideoCapture cap(device_id);
  if (!cap.isOpened()) {
    ROS_ERROR("Failed to open /dev/video%d", device_id);
    return 1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  // Prefer MJPG for higher effective framerate on USB cameras
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

  double actual_w = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  double actual_h = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  ROS_INFO("Camera /dev/video%d opened: %.0fx%.0f", device_id, actual_w, actual_h);

  // --- publishers ---
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_raw = it.advertise("camera/image", 1);

  ros::Publisher pub_compressed;
  if (publish_compressed_only) {
    pub_compressed = nh.advertise<sensor_msgs::CompressedImage>("camera/image/compressed", 1);
  }

  cv::Mat frame;
  int seq = 0;

  while (ros::ok()) {
    if (!cap.read(frame) || frame.empty()) {
      ROS_WARN_THROTTLE(2.0, "Camera read failed");
      ros::spinOnce();
      continue;
    }

    ros::Time now = ros::Time::now();
    auto header = std_msgs::Header();
    header.stamp = now;
    header.frame_id = frame_id;
    header.seq = seq++;

    if (publish_compressed_only) {
      // Direct compressed publish (JPEG)
      sensor_msgs::CompressedImage msg;
      msg.header = header;
      msg.format = "jpeg";
      std::vector<uchar> buf;
      cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 85});
      msg.data.assign(buf.begin(), buf.end());
      pub_compressed.publish(msg);
    } else {
      // Publish via image_transport (raw + auto-compressed topics)
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      pub_raw.publish(msg);
    }

    ros::spinOnce();
  }

  cap.release();
  return 0;
}