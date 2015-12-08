#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/lexical_cast.hpp>

#include <sensor_msgs/image_encodings.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace zed_ros {

class CameraDriver
{
  ros::NodeHandle priv_nh_;
  ros::NodeHandle camera_nh_;

  std::string camera_name_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  ros::NodeHandle nh_l_;
  image_transport::CameraPublisher image_pub_l_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_l_;

  ros::NodeHandle nh_r_;
  image_transport::CameraPublisher image_pub_r_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_r_;

  cv::VideoCapture cap_;

  std::string video_device_name_;
  int image_width_;
  int image_height_;
  int framerate_;

 public:
  CameraDriver(ros::NodeHandle priv_nh, ros::NodeHandle camera_nh):
      priv_nh_(priv_nh),
      camera_nh_(camera_nh),
      camera_name_("zed"),
      it_(new image_transport::ImageTransport(camera_nh_)) {}

  void setup() {
    priv_nh_.param("video_device", video_device_name_, std::string("/dev/video0"));
    priv_nh_.param("image_width", image_width_, 1280);
    priv_nh_.param("image_height", image_height_, 720);
    priv_nh_.param("framerate", framerate_, 30);

    std::set<int> framerates;
    if (image_width_ == 2208 && image_height_ == 1242) {
      framerates.insert(15);
    } else if (image_width_ == 1920 && image_height_ == 1080) {
      framerates.insert(15);
      framerates.insert(30);
    } else if (image_width_ == 1280 && image_height_ == 720) {
      framerates.insert(15);
      framerates.insert(30);
      framerates.insert(60);
    } else if (image_width_ == 640 && image_height_ == 480) {
      framerates.insert(15);
      framerates.insert(30);
      framerates.insert(60);
      framerates.insert(100);
    } else {
      ROS_WARN("Invalid image resolution selected %ix%i, defaulting to 1280x720", image_width_, image_height_);
       image_width_ = 1280; 
       image_height_ = 720;
    }

    if (framerates.count(framerate_) == 0) {
      ROS_WARN("Invalid framerate selected %i, defaulting to %i", framerate_, *(framerates.begin()));
      framerate_ = *framerates.begin();
    }

    boost::filesystem::path path(video_device_name_);
    std::string target = boost::filesystem::canonical(path).string();
    boost::regex exp(".video(\\d+)");
    boost::smatch match;
    boost::regex_search(target, match, exp);
    std::cout << std::string(match[1].first, match[1].second) << std::endl;
    int id = boost::lexical_cast<int>(std::string(match[1].first, match[1].second));

    cap_ = cv::VideoCapture(id);
    cap_.set(CV_CAP_PROP_FRAME_WIDTH, image_width_*2);
    cap_.set(CV_CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(CV_CAP_PROP_FPS, framerate_);

    if(!cap_.isOpened()) {
      ROS_ERROR("Failed to open video capture");
    }

    nh_l_ = ros::NodeHandle(camera_nh_, "left");
    cam_info_l_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh_l_));
    cam_info_l_->setCameraName(camera_name_ + "_" + boost::lexical_cast<std::string>(image_width_) + "_left");
    image_pub_l_ = it_->advertiseCamera("left/image_raw", 1);

    nh_r_ = ros::NodeHandle(camera_nh_, "right");
    cam_info_r_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(nh_r_));
    cam_info_r_->setCameraName(camera_name_ + "_" + boost::lexical_cast<std::string>(image_width_) + "_right");
    image_pub_r_ = it_->advertiseCamera("right/image_raw", 1);
  }


  ~CameraDriver() {
  }

  void poll() {
    cv::Mat frame;

    if(!cap_.isOpened())
      return;

    cap_ >> frame;

    if (frame.rows == 0 && frame.cols == 0)
      return;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "zed_optical";

    sensor_msgs::CameraInfoPtr ci_left(new sensor_msgs::CameraInfo(cam_info_l_->getCameraInfo()));
    ci_left->header = header;
    sensor_msgs::ImagePtr im_left  = cv_bridge::CvImage(header, "bgr8", frame(cv::Range::all(), cv::Range(0, image_width_))).toImageMsg();
    image_pub_l_.publish(im_left, ci_left);

    sensor_msgs::CameraInfoPtr ci_right(new sensor_msgs::CameraInfo(cam_info_r_->getCameraInfo()));
    ci_right->header = header;
    sensor_msgs::ImagePtr im_right = cv_bridge::CvImage(header, "bgr8", frame(cv::Range::all(), cv::Range(image_width_, image_width_*2))).toImageMsg();
    image_pub_r_.publish(im_right, ci_right);
  }
};


class CameraDriverNodelet: public nodelet::Nodelet
{
public:
  CameraDriverNodelet():
    running_(false)
  {}

  ~CameraDriverNodelet() {
    if (running_) {
      NODELET_INFO("Shutting down driver thread");
      running_ = false;
      deviceThread_->join();
    }
  }

private:
  virtual void onInit();
  virtual void devicePoll();
  volatile bool running_;
  boost::shared_ptr<zed_ros::CameraDriver> driver_;
  boost::shared_ptr<boost::thread> deviceThread_;
};

void CameraDriverNodelet::onInit()
{
  ros::NodeHandle priv_nh(getPrivateNodeHandle());
  ros::NodeHandle nh(getNodeHandle());
  ros::NodeHandle camera_nh(nh);

  driver_.reset(new zed_ros::CameraDriver(priv_nh, camera_nh));
  driver_->setup();
  running_ = true;
  deviceThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CameraDriverNodelet::devicePoll, this)));
}

void CameraDriverNodelet::devicePoll() {
  while(running_) {
    driver_->poll();
  }
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_ros::CameraDriverNodelet, nodelet::Nodelet);

