#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>
#include <boost/thread/mutex.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string SLOPE_WINDOW = "slope window";


namespace enc = sensor_msgs::image_encodings;

class PlaneDetect
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	sensor_msgs::ImageConstPtr kinect_depthimg_msg, old_kinect_depthimg_msg;
	boost::mutex m;

	public:
	PlaneDetect()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
		  &PlaneDetect::planeDetect_callback, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW);
	}
	~PlaneDetect()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}
	void planeDetect_callback(const sensor_msgs::ImageConstPtr& msg)
	{

		m.lock ();
		kinect_depthimg_msg = msg;
		m.unlock ();

		if (!kinect_depthimg_msg || kinect_depthimg_msg == old_kinect_depthimg_msg)
			return;
		old_kinect_depthimg_msg = kinect_depthimg_msg;

		//change ros image to cv image
		cv_bridge::CvImagePtr cv_depthptr;
		try
		{
			cv_depthptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		int t_margin=20;

		//process depth cv image
		cv::Mat depth_clean(cv_depthptr->image.rows, cv_depthptr->image.cols, CV_32FC1);
		cv::Mat img(cv_depthptr->image.rows, cv_depthptr->image.cols, CV_8UC1);
		for(int i = t_margin; i < cv_depthptr->image.rows-t_margin; i++)
		{
			float* Di = cv_depthptr->image.ptr<float>(i);
			float* Ii = depth_clean.ptr<float>(i);
			char* Ivi = img.ptr<char>(i);
			for(int j = t_margin; j < cv_depthptr->image.cols-t_margin; j++)
			{
				if(Di[j] > 0.0f)
				{
					Ii[j] = Di[j];
					Ivi[j] = (char) (255*((Di[j])/(5.5))); // some suitable values..
				}
				else
				{
					Ii[j] = 0.0f;
					Ivi[j] = 0;
				}
			}
		}

		//get slope of row direction

		cv::Mat plane_slope_row(cv_depthptr->image.rows, cv_depthptr->image.cols, CV_8UC3);
		static int t_length_partition=5;
		static int t_num_row_partition=(640-2*t_margin)/t_length_partition;
/*
		int t_color_r=0, t_color_g=0, t_color_b=0;

		for (int i = t_margin; i < cv_depthptr->image.rows-t_margin-1; i++)
		{
			t_color_r=(t_color_r+30) % 255;
			t_color_g=(t_color_g+100)% 255;
			for(int j_index=0; j_index<t_num_row_partition; j_index++)
			{
				t_color_b=(t_color_b+40)%255;
				cv::line(plane_slope_row,
						cv::Point(t_margin+j_index*t_length_partition,i),cv::Point(t_margin+(j_index+1)*t_length_partition,i),
						cv::Scalar(t_color_r,t_color_g,t_color_b),1,CV_AA);
			}
		}
*/
		cv::Mat var(480,640,CV_32FC1);
		float var_max=0;
		float var_min=9999;
		float var_aver=0;

		for (int i = t_margin; i < cv_depthptr->image.rows-t_margin; i++)
		{
			for(int j_index=0; j_index<t_num_row_partition; j_index++)
			{
				int n=0;
				float sumx=0;
				float sumy=0;
				float sumxx=0;
				float sumxy=0;
				float sumyy=0;
				for(int k=0;k<t_length_partition;k++)
				{
					float x=k;
					float y=depth_clean.ptr<float>(i)[t_margin+(j_index*t_length_partition+k)];
					n++;
					sumx+=x;
					sumy+=y;
					sumxx+=x*x;
					sumyy+=y*y;
					sumxy+=x*y;

				}
//				ROS_INFO("n=%d,sumx=%f,sumy=%f,sumxx=%f,sumyy=%f,sumxy=%f",n,sumx,sumy,sumxx,sumyy,sumxy);

				float a=(n*sumxy-sumx*sumy)/(n*sumxx-sumx*sumx);
				float b=(sumy-a*sumx)/n;
				var.ptr<float>(i)[j_index]=sumyy+a*a*sumxx+b*b*n-2*a*sumxy-2*b*sumy+2*a*b*sumx;
				if(var.ptr<float>(i)[j_index]>var_max) var_max=var.ptr<float>(i)[j_index];
				if(var.ptr<float>(i)[j_index]<var_min) var_min=var.ptr<float>(i)[j_index];
				var_aver+=var.ptr<float>(i)[j_index];
//				ROS_INFO("a=%f,b=%f,var=%f",a,b,var.ptr<float>(i)[j_index]);
			}
		}

		var_aver=var_aver/(t_length_partition*(cv_depthptr->image.rows-2*t_margin));
		ROS_INFO("%f, %f, %f",var_min, var_aver, var_max);

		int t_color_r=0, t_color_g=0, t_color_b=0;

		for (int i = t_margin; i < cv_depthptr->image.rows-t_margin-1; i++)
		{

			for(int j_index=0; j_index<t_num_row_partition; j_index++)
			{

				t_color_r=255*(1-var.ptr<float>(i)[j_index]*(var_max-var_min));
				if (var.ptr<float>(i)[j_index]<var_aver) t_color_g=100;
				//t_color_b=(t_color_b+40)%255;
				cv::line(plane_slope_row,
						cv::Point(t_margin+j_index*t_length_partition,i),cv::Point(t_margin+(j_index+1)*t_length_partition,i),
						cv::Scalar(t_color_b,t_color_g,t_color_r),1,CV_AA);
			}
		}



//		ROS_INFO("%d",t_num_row_partition);

//			ROS_INFO("%d,%d",cv_depthptr->image.rows,cv_depthptr->image.cols);
//			ROS_INFO("%f,      %f,      %f",depth_clean.ptr<float>(320)[240],img.ptr<float>(320)[240],cv_depthptr->image.ptr<float>(320)[240]);
		cv_bridge::CvImage cvres_out;
		cvres_out.header = cv_depthptr->header;
		cvres_out.encoding = enc::TYPE_32FC1;
		cvres_out.image = depth_clean;
		image_pub_.publish(cvres_out.toImageMsg());


		// display
		cv::imshow(OPENCV_WINDOW, img);
		cv::imshow(SLOPE_WINDOW, plane_slope_row);

		//cv::imshow(OPENCV_WINDOW, depth_clean);
		cv::waitKey(3);


	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	PlaneDetect pd;
	ros::spin();
	return 0;
}
