#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string GRAY_WINDOW = "gray window";
static const std::string EDGE_WINDOW = "edge window";
static const std::string LINE_WINDOW = "line window";



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(GRAY_WINDOW);
    cv::namedWindow(EDGE_WINDOW);
    cv::namedWindow(LINE_WINDOW);


  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(GRAY_WINDOW);
    cv::destroyWindow(EDGE_WINDOW);
    cv::destroyWindow(LINE_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }




    cv::Mat edge_image;
    cv_ptr->image.copyTo(edge_image);
//    cv::Mat gray_image;


 //   std::vector<cv::Mat> channels;
//    cv::Mat hsv;
//    cv::cvtColor(edge_image, hsv, CV_RGB2HSV );
 //   cv::split(hsv, channels);
//    gray_image = channels[0];

    cv::Mat contours;
    double low_threshold=100;
    double threshold_ratio=3;
    cv::Mat cdst;
    cv::Canny(edge_image,contours,low_threshold,low_threshold*threshold_ratio);
    cv::cvtColor(contours, cdst, CV_GRAY2BGR);

    cv::vector<cv::Vec4i> lines;
    cv::HoughLinesP(contours, lines, 1, CV_PI/180, 100, 100, 10 );

    for (size_t i=0;i<lines.size();i++)
    {
    	cv::Vec4i l=lines[i];
    	cv::line(cdst, cv::Point(l[0],l[1]), cv::Point(l[2],l[3]),cv::Scalar(0,0,255),3,CV_AA);
    }

    ROS_INFO("%d",lines.size());
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
//    cv::imshow(GRAY_WINDOW, gray_image);
  //  cv::waitKey(3);
    cv::imshow(EDGE_WINDOW, contours);
    cv::waitKey(3);
    cv::imshow(LINE_WINDOW, cdst);
    cv::waitKey(3);



    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
