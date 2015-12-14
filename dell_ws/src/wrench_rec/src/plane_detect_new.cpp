#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>
#include <boost/thread/mutex.hpp>

#include <math.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

static const std::string WINDOW1 = "window1";
static const std::string WINDOW2 = "window2";
static const std::string WINDOW3 = "window3";
static const std::string WINDOW4 = "window4";
static const std::string WINDOW5 = "window5";
static const int ROW_NUM=480;
static const int COL_NUM=640;
static const int CONCERNED_RANGE=100;
//static const int FINDING_RANGE=CONCERNED_RANGE+50;


namespace enc = sensor_msgs::image_encodings;

class PlaneDetect
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	image_transport::Subscriber rgb_image_sub;

	sensor_msgs::ImageConstPtr kinect_depthimg_msg, old_kinect_depthimg_msg;
	boost::mutex m;
	int z_value;

	public:
	PlaneDetect()
		: it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
		  &PlaneDetect::planeDetect_callback, this);
//		rgb_image_sub =it_.subscribe("/camera/rgb/image_color", 1,
//				&PlaneDetect::rgb_callback,this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(WINDOW1);
		cv::namedWindow(WINDOW2);
		cv::namedWindow(WINDOW3);
//		cv::namedWindow(WINDOW4);
//		cv::namedWindow(WINDOW5);
	}
	~PlaneDetect()
	{
		cv::destroyWindow(WINDOW1);
		cv::destroyWindow(WINDOW2);
		cv::destroyWindow(WINDOW3);
//		cv::destroyWindow(WINDOW4);
//		cv::destroyWindow(WINDOW5);
	}

	void rgb_callback(const sensor_msgs::ImageConstPtr& msg)
	{

		cv_bridge::CvImagePtr cv_rgbptr;
				try
				{
					cv_rgbptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				}
				catch (cv_bridge::Exception& e)
				{
					ROS_ERROR("cv_bridge exception: %s", e.what());
					return;
				}

				cv::imshow(WINDOW4, cv_rgbptr->image);
				cv::waitKey(3);

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

		Process(cv_depthptr);
	}

	void Process(cv_bridge::CvImagePtr _cvimageptr)
	{


		//convert depth pixel to xyz
		cv::Mat matx(ROW_NUM,COL_NUM,CV_32FC1);
		cv::Mat maty(ROW_NUM,COL_NUM,CV_32FC1);
		cv::Mat matz(ROW_NUM,COL_NUM,CV_32FC1);

		Convert_depth_image_to_xyz_data(_cvimageptr->image, &matx,&maty,&matz);
		//

		//test for the mid line
		cv::Mat image_color(ROW_NUM,COL_NUM,CV_8UC3);
		Test_line_derivative(240,matx,matz,&image_color);
		//


		cv::Mat balanced_image(ROW_NUM,COL_NUM,CV_32FC1);

		Get_balanced_image(matx,matz,&balanced_image);
		cv::Mat object_image(ROW_NUM,COL_NUM,CV_8UC3);
		cv::Mat object_number(ROW_NUM,COL_NUM,CV_32FC1);

		Display_object(balanced_image,&object_image,&object_number);

		for (int i=CONCERNED_RANGE; i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			_cvimageptr->image.ptr<float>(i)[CONCERNED_RANGE]=0;
			_cvimageptr->image.ptr<float>(i)[COL_NUM-CONCERNED_RANGE]=0;
		}
		for (int j=CONCERNED_RANGE; j<COL_NUM-CONCERNED_RANGE;j++)
		{
			_cvimageptr->image.ptr<float>(CONCERNED_RANGE)[j]=0;
			_cvimageptr->image.ptr<float>(ROW_NUM-CONCERNED_RANGE)[j]=0;
		}

		cv::imshow(WINDOW2, _cvimageptr->image);
		cv::imshow(WINDOW1, image_color);
		cv::imshow(WINDOW3, object_image);
//		cv::imshow(WINDOW4, image_colored_depth);
		cv::waitKey(3);

	}
	void Convert_depth_image_to_xyz_data(cv::Mat _input_depth_image,
			cv::Mat* _x_output, cv::Mat* _y_output, cv::Mat* _z_output)
	{
		//cncel error depth
		//calculate z_output
		static const float fx_d=594.21434211923247;
		static const float fy_d=591.04053696870778;
		static const float cx_d=339.30780975300314;
		static const float cy_d=242.73913761751615;
		cv::Mat* t_mat;
		t_mat = &_input_depth_image;

		int counter=1;
		for(int i=CONCERNED_RANGE; i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{
				if(t_mat->ptr<float>(i)[j]>0)
				{
					_z_output->ptr<float>(i)[j]=t_mat->ptr<float>(i)[j];
					counter++;
				}
				else
				{
					_z_output->ptr<float>(i)[j]=t_mat->ptr<float>(i)[j-1];
					if(!(_z_output->ptr<float>(i)[j]))
					{
						_z_output->ptr<float>(i)[j]=t_mat->ptr<float>(i-1)[j];
					if(!(_z_output->ptr<float>(i)[j]))
						{
							_z_output->ptr<float>(i)[j]=t_mat->ptr<float>(i)[j+1];
							if(!(_z_output->ptr<float>(i)[j]))
							{
								_z_output->ptr<float>(i)[j]=t_mat->ptr<float>(i+1)[j];
							}
							else
							{
								_z_output->ptr<float>(i)[j]=0;
							}
						}
					}

				}
			}//for j
		}//for i




		if (counter<(ROW_NUM-2*CONCERNED_RANGE)*(COL_NUM-2*CONCERNED_RANGE)/2)
		{
//			printf("not enough depth data,");
		}
//		printf("counter=%d, z_done\n",counter);

		//calculate x and y _output
		//P3D.x = (x_d - cx_d) * depth(x_d,y_d) / fx_d
		//P3D.y = (y_d - cy_d) * depth(x_d,y_d) / fy_d
		//P3D.z = depth(x_d,y_d)

		for(int i=CONCERNED_RANGE; i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{
				_x_output->ptr<float>(i)[j]=(j-cx_d)*_z_output->ptr<float>(i)[j]/fx_d;
				_y_output->ptr<float>(i)[j]=(j-cy_d)*_z_output->ptr<float>(i)[j]/fy_d;
			}

		}

	}// void Convert_depth_image_to_depth_data(cv::Mat _input_depth_image, cv::Mat* _x_output, cv::Mat* _y_output, cv::Mat* _z_output)

	void Test_line_derivative(int _row_index, cv::Mat _i_x, cv::Mat _i_z, cv::Mat* _o_color_image)
	{
		float* Xi=_i_x.ptr<float>(_row_index);
		float* Zi=_i_z.ptr<float>(_row_index);

		float Bi[COL_NUM];
		float Ki[COL_NUM];
//		float Znewi[COL_NUM];

		float sum_z=0;
		float sum_ksqr=0;
		int counter=1;
		int counter_ksqr=1;
		float min_z=10000;
		float max_z=-10000;
		int minimum_depth_value=100;
		int largest_k_index=0;
		float largest_k=0;

		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{
			if(Zi[j]>minimum_depth_value)
			{
//				Znewi[j]=Zi[j];
				sum_z+=Zi[j];
				counter++;
				if(Zi[j]>max_z) max_z=Zi[j];
				if(Zi[j]<min_z) min_z=Zi[j];
				int z=(int)Zi[j];
				//original line red

//				cout<<"j="<<j<<","<<"Zi[j]="<<z<<endl;
				cv::circle(*_o_color_image, cv::Point(j,z%480),1,cv::Scalar(0,0,255));
				if(Zi[j-1]>minimum_depth_value)
				{
					Ki[j]=Zi[j]-Zi[j-1];
					counter_ksqr++;
					sum_ksqr+=Ki[j]*Ki[j];
					if(Ki[j]>largest_k)
					{
						largest_k=Ki[j];
						largest_k_index=j;
					}
				}
				//				cout<<"j="<<j<<","<<"Ki[j]="<<Ki[j]<<endl;

			}
		}//for
		float aver_ksqr=sum_ksqr/counter_ksqr;
		float aver_z=sum_z/counter;
		//		cout<<"aver_ksqr="<<aver_ksqr<<",counter_ksqr="<<counter_ksqr<<endl;
		//		cout<<"largest_ksqr="<<largest_k<<",largest_ksqr_index="<<largest_k_index<<endl;

		float sum_k=0;
		float counter_k=1;


//		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		for (int j=CONCERNED_RANGE;j<largest_k_index;j++)
		{
			if((Ki[j]*Ki[j])<aver_ksqr+1)
			{
				sum_k+=Ki[j];
				counter_k++;
			}
		}
		float aver_k=sum_k/counter_k;
//		cout<<"aver_k="<<aver_k<<",counter_k="<<counter_k<<endl;
		int t_z=(int)(aver_z);
		//average line - green
		cv::line(*_o_color_image, cv::Point(0,t_z%480),
				cv::Point(639,t_z%480),cv::Scalar(0,255,0),1);
		float t_b=aver_z-240*aver_k;
		int fitting_line_z1=t_b;
		int fitting_line_z2=(int)(639*aver_k+t_b);
//		cout<<"fitting_line_z1="<<fitting_line_z1<<",fitting_line_z2="<<fitting_line_z2<<endl;
		cv::line(*_o_color_image, cv::Point(0,fitting_line_z1%480),
				cv::Point(639,fitting_line_z2%480),cv::Scalar(255,0,0),1);

		float t_Bi[COL_NUM];
		float sum_t_bi=0;
		float counter_t_bi=1;
		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{
//			if(Zi[j]>minimum_depth_value)
			if(j<largest_k_index)
			{
				t_Bi[j]=Zi[j]-aver_k*Xi[j]-t_b;
				sum_t_bi+=t_Bi[j];
				counter_t_bi++;
			}
			else
			{t_Bi[j]=0;}
/*
			int z=t_Bi[j]+aver_z;
			cv::circle(*_o_color_image,
					cv::Point(j,z%480),2,cv::Scalar(255,0,255));
					*/
		}
		float aver_t_bi=sum_t_bi/counter_t_bi;
//		cout<<aver_t_bi<<","<<counter_t_bi<<endl;


		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{
//			if(t_Bi[j]<aver_t_bi)
			if(j<largest_k_index)
			{
				Bi[j]=t_Bi[j];
			}
			else
			{
				Bi[j]=aver_t_bi;
			}

			int z=t_Bi[j]+aver_z;
		    	cv::circle(*_o_color_image,
							cv::Point(j,z%480),3,cv::Scalar(0,255,255));
		}

/*


		for(int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{
			Bi[j]=Zi[j]-k*Xi[j]-b;
			if (Znewi[j]==0)
			{
				Bi[j]=0;
			}
			int z=Bi[j]+aver_z;
			//balanced line - pink
			cv::circle(*_o_color_image,
					cv::Point(j,z%480),2,cv::Scalar(255,0,255));

		}
		*/
	}//Test_line_diretive

	void Get_balanced_image(cv::Mat _i_x, cv::Mat _i_z, cv::Mat* _o_balanced_image)
	{
		float b_sum=0;
		int b_counter=1;
		for(int i=CONCERNED_RANGE;i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			float* Xi=_i_x.ptr<float>(i);
			float* Zi=_i_z.ptr<float>(i);
			float* Bi=_o_balanced_image->ptr<float>(i);
			float sum_z=0;
			int counter=1;
			for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{if(Zi[j]>0)	{sum_z+=Zi[j];counter++;}}
			float aver_z=sum_z/counter;

			Eigen::MatrixXf matrix_A(3,1);
			Eigen::MatrixXf matrix_X(3,COL_NUM-2*CONCERNED_RANGE);
			Eigen::MatrixXf matrix_L(3,3);

			for(int j=0;j<COL_NUM-2*CONCERNED_RANGE;j++)
			{
				float x,z,d;
				if(Zi[j+CONCERNED_RANGE]>0)

				{
					x=Xi[j+CONCERNED_RANGE];
					z=Zi[j+CONCERNED_RANGE];
					d=1;
				}
				else {x=0;z=0;d=0;}
				matrix_X.col(j)<<x,z,d;
			}
			matrix_L=matrix_X*matrix_X.transpose();
			Eigen::EigenSolver<Eigen::MatrixXf> es(matrix_L);

			float hmin=0;
			int hindex=0;
			for (int j=0;j<3;j++)
			{
				std::complex<float> lambda=es.eigenvalues()[j];
				if(lambda.imag()==0)
				{
					if(hmin==0) {hmin=lambda.real();hindex=j;}
					else
					{
						if(lambda.real()<hmin)	{hmin=lambda.real();hindex=j;}
					}
				}
			}


			//output
			//if(hmin==0) {cout << "hmin=0,error, row " <<i<< endl;}

			Eigen::VectorXcf hvector=es.eigenvectors().col(hindex);
			Eigen::VectorXf hvector_real=hvector.real();
			float k,b;
			k=-hvector_real[0]/hvector_real[1];
			b=-hvector_real[2]/hvector_real[1];
//			cout<<b<<endl;
			b_sum+=b;
			b_counter++;

			for(int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{
				Bi[j]=-Zi[j]+k*Xi[j]+b;
			}

//			cout<<"lambda"<<hmin<<"k"<<k<<"b"<<b<<endl;
		}//i
		int b_aver=b_sum/b_counter;
		z_value=b_aver;
//		cout<<"depth from sensor to plane:"<<b_aver<<endl;

	}//Get_balanced_image

	void Display_object(cv::Mat _i_balanced_image, cv::Mat* _o_object_image, cv::Mat* _o_object_number)
	{

		for (int i=CONCERNED_RANGE-1;i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			_o_object_number->ptr<int>(i)[CONCERNED_RANGE-1]=0;
			_o_object_number->ptr<int>(i)[COL_NUM-CONCERNED_RANGE+1]=0;
		}
		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{
			_o_object_number->ptr<int>(CONCERNED_RANGE-1)[j]=0;
			_o_object_number->ptr<int>(ROW_NUM-CONCERNED_RANGE+1)[j]=0;
		}

		int object_index=0;

		float object_d_sum=0;
		int object_d_counter=1;
		for(int i=CONCERNED_RANGE;i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			float sum_sqr=0;
			int counter=1;
			float max=0;

			for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{
				if ((_i_balanced_image.ptr<float>(i)[j]>0)&&(_i_balanced_image.ptr<float>(i)[j]<200))
				{
					if(_i_balanced_image.ptr<float>(i)[j]>5)
					{
						object_d_sum+=_i_balanced_image.ptr<float>(i)[j];
						object_d_counter++;
					}
					sum_sqr+=_i_balanced_image.ptr<float>(i)[j]*_i_balanced_image.ptr<float>(i)[j];
					counter++;
					if(_i_balanced_image.ptr<float>(i)[j]>max) max=_i_balanced_image.ptr<float>(i)[j];
				}
			}//j

			float aver=sqrt(sum_sqr/counter);
//			cout <<"i"<<i<<"aver"<<aver<<"max"<<max<<endl;

			for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{
				int r=0,g=0,b=0;
				if((aver<5)||(_i_balanced_image.ptr<float>(i)[j]<2))
				{
					b=255;
					_o_object_number->ptr<int>(i)[j]=0;
				}
				else if(max>5)
				{
					r=255*((float)_i_balanced_image.ptr<float>(i)[j]/max);
					if (_o_object_number->ptr<int>(i-1)[j]!=0)
					{_o_object_number->ptr<int>(i)[j]=_o_object_number->ptr<int>(i-1)[j];}
					else if (_o_object_number->ptr<int>(i-2)[j]!=0)
					{_o_object_number->ptr<int>(i)[j]=_o_object_number->ptr<int>(i-2)[j];}
//					else if(_o_object_number->ptr<int>(i-1)[j-1]!=0)
//					{_o_object_number->ptr<int>(i)[j]=_o_object_number->ptr<int>(i-1)[j-1];}
//					else if(_o_object_number->ptr<int>(i-1)[j+1]!=0)
//					{_o_object_number->ptr<int>(i)[j]=_o_object_number->ptr<int>(i-1)[j+1];}
//					else if(_o_object_number->ptr<int>(i)[j-1]!=0)
//					{_o_object_number->ptr<int>(i)[j]=_o_object_number->ptr<int>(i)[j-1];}
					else
					{
							object_index++;
							_o_object_number->ptr<int>(i)[j]=object_index;
					}
				}

				if(_o_object_number->ptr<int>(i)[j]!=0){g=(_o_object_number->ptr<int>(i)[j]*100)%255;}

//				cv::circle(*_o_object_image, cv::Point(j,i),1,cv::Scalar(b,g,r));
			}//j
		}//i
		int object_d_aver=object_d_sum/object_d_counter;
//		cout<<"depth of wrenches above plane:"<<object_d_aver<<endl;

//		cout << "object_num" << object_index<<endl;
		int ymin[COL_NUM];
		int ymax[COL_NUM];
		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{//j
			ymin[j]=0;
			ymax[j]=0;
			int counter=1;
			int sum=0;
			for(int i=CONCERNED_RANGE;i<ROW_NUM-CONCERNED_RANGE;i++)
			{
				if(_o_object_number->ptr<int>(i)[j]>0)
				{
					sum+=i;
					counter++;
				}
			}//i
			if(counter>10)
			{

				int y_aver=sum/counter;
				int y_min=y_aver-counter/2;
				int y_max=y_aver+counter/2;
				int new_counter=1;
				for (int i=y_min;i<y_max;i++)
				{
					if(_o_object_number->ptr<int>(i)[j]!=0)
					{
						new_counter++;
					}
				}
				float ratio=float(new_counter)/float(counter);
				if(ratio>0.6)
				{
					ymin[j]=y_min;
					ymax[j]=y_max;
//					cv::circle(*_o_object_image, cv::Point(j,y_min),2,cv::Scalar(255,255,255));
//					cv::circle(*_o_object_image, cv::Point(j,y_max),2,cv::Scalar(255,255,255));
				}

//				cout<<"j"<<j<<"y_min"<<y_min<<"y_max"<<y_max<<endl;
			}
		}//j
//		cv::putText(*_o_object_image,"123451232", cv::Point(320,240),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255));
		int new_object_index=0;
		int object_range_min[COL_NUM];
		int object_range_max[COL_NUM];
		bool object_start_flag=false;
		object_range_min[new_object_index]=0;
		object_range_max[new_object_index]=0;
		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{

			if(ymax[j]!=0)
			{
				if(object_start_flag==false)
				{
					object_range_min[new_object_index]=j;
					object_range_max[new_object_index]=j;
					object_start_flag=true;

				}
				else
				{
					object_range_max[new_object_index]=j;

				}
			}
			else
			{
				if(object_start_flag==true)
				{
					object_start_flag=false;
					new_object_index++;
					object_range_min[new_object_index]=0;
					object_range_max[new_object_index]=0;
				}
			}
		}//for j


		new_object_index++;
		object_range_min[new_object_index]=0;
		object_range_max[new_object_index]=0;
//		cout<<"new_object_index"<<new_object_index<<endl;

		int set_object_index=0;
		int object_counter[20];
		for (int i=0;i<20;i++) {object_counter[i]=0;}
		if (new_object_index<20)
		{
			while (object_range_max[set_object_index]>0)
			{
//				std::ostringstream str;
				int objectk_x_min=object_range_min[set_object_index];
				int objectk_x_max=object_range_max[set_object_index];
				int objectk_counter=1;
				for (int j=objectk_x_min;j<objectk_x_max+1;j++)
				{
					for (int i=ymin[j];i<ymax[j]+1;i++)
					{
						if(_i_balanced_image.ptr<float>(i)[j]>0) {objectk_counter++;}
					}
				}

				if(objectk_counter>10) {object_counter[set_object_index]=objectk_counter;}
//				str<<"("<<set_object_index<<"),"<<objectk_counter;
//				cv::Point point=cv::Point(object_range_max[set_object_index],ymin[object_range_max[set_object_index]]);

//				cv::putText(*_o_object_image,str.str(),point ,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(128,128,128),3);
				set_object_index++;
			}
		}
		int object_order[20];
		for(int i=0;i<set_object_index;i++)
		{
			object_order[i]=i;
		}

		for(int i=0;i<set_object_index-1;i++)
		{
			for(int j=0;j<set_object_index-1;j++)
			{
				if(object_counter[object_order[j]]<object_counter[object_order[j+1]])
				{
					int temp;
					temp=object_order[j];
					object_order[j]=object_order[j+1];
					object_order[j+1]=temp;
				}
			}
		}

//		for(int i=0;i<set_object_index;i++)
		int max_object=5;
		int green_difference_between_objects=(int)(240/max_object);
		if (set_object_index<max_object)
		{
			max_object=set_object_index;
		}
		for(int k=0;k<max_object;k++)
		{
		
			int index=object_order[k];
			int sum=0;
//			*_o_object_image = cv_rgbptr->image.clone();x`
			int y_min_sum=0;
			int y_max_sum=0;
			int min_y=600;
			int max_y=40;
			for(int j=object_range_min[index];j<object_range_max[index]+1;j++)
			{

				cv::circle(*_o_object_image, cv::Point(j,ymin[j]),2,cv::Scalar(255,255,255));
				cv::circle(*_o_object_image, cv::Point(j,ymax[j]),2,cv::Scalar(255,255,255));
				sum+=ymin[j]+ymax[j];
				int g=k*green_difference_between_objects;
				y_min_sum+=ymin[j];
				y_max_sum+=ymax[j];
				if (ymin[j]<min_y) min_y=ymin[j];
				if (ymax[j]>max_y) max_y=ymax[j];
				for(int i=ymin[j];i<ymax[j];i++)
				{

					cv::circle(*_o_object_image, cv::Point(j,i),1,cv::Scalar(128,g,0));
				}
			}
			std::ostringstream str;
			str<<"("<<k<<")";
			int mid_x=(object_range_max[index]+object_range_min[index])/2;
			int mid_y=(sum)/(2*(1+object_range_max[index]-object_range_min[index]));
//			min_y=(y_min_sum)/(1+object_range_max[index]-object_range_min[index]);
//			max_y=(y_max_sum)/(1+object_range_max[index]-object_range_min[index]);
			int min_x=object_range_min[index];
			int max_x=object_range_max[index];
				int x_value=(mid_x-320)*z_value/500;
				int y_value=(240-mid_y)*z_value/500;
//	cout<<"order:"<<object_order[k]<<",size"<<object_counter[object_order[k]]<<"@("<<x_value<<","<<y_value<<")"<<endl;

//			cout<<"minx,miny"<<min_x<<","<<min_y<<",maxx,maxy"<<max_x<<","<<max_y<<endl;

			cv::Point point=cv::Point(mid_x,mid_y);
			cv::putText(*_o_object_image,str.str(),point ,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),3);
			cv::line(*_o_object_image,cv::Point(mid_x-15,mid_y),cv::Point(mid_x+15,mid_y),cv::Scalar(255,255,0),2);
			cv::line(*_o_object_image,cv::Point(mid_x,mid_y-15),cv::Point(mid_x,mid_y+15),cv::Scalar(255,255,0),2);
			cv::rectangle(*_o_object_image,cv::Point(min_x,min_y),cv::Point(max_x,max_y),cv::Scalar(255,255,0),1);
			
			if(k==max_object-1)
			  {


   cout<<"the smallest wrench of all the "<<max_object<<" wrenches is the number " <<(max_object-1)
				<<" wrench, the position is @("<<x_value<<","<<y_value<<","<<z_value<<"), defining the center of the kinect is (0,0,0) position, and left is the positive x direction, up is the positive y direction, forward is the positive z direction"<<endl;
			  }



		}//k  number of object displayed
						cv::imshow(WINDOW3, *_o_object_image);
						cv::waitKey(3);

	}//function



	bool Get_balanced_depth_image(cv::Mat _original_depth_image, cv::Mat _x, cv::Mat _y, cv::Mat _z,
								float _a1, float _a2, float _a3, float _a4,
								cv::Mat* _o_balanced_depth_image)
	{
		//a1x+a2y+a3z+a4=0
		//if a3=0, the plane pass z-axis, which is impossible in this case;
		//point in the plane z=-(a4-a1x-a2y)/a3
		//point above the plane: z<-(a4-a1x-a2y)/a3
		//point below the plane; z>-(a4-a1x-a2y)/a3
		//let z_new=(a1x+a2y+a3z-a4)/a3
		//z_new=0, on the plane, z_new>0 above the plane, z_new<0 below the plane

		if(_a3==0) {printf("a3=0,error");return 0;}

		int counter=1;
		int sum=0;

		for(int i=CONCERNED_RANGE; i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{
				if(_original_depth_image.ptr<float>(i)[j]>0)
				{
					_o_balanced_depth_image->ptr<float>(i)[j]=(_a1*_x.ptr<float>(i)[j]+
															_a2*_y.ptr<float>(i)[j]+
															_a3*_original_depth_image.ptr<float>(i)[j]-
															_a4)/_a3;
					sum+=_o_balanced_depth_image->ptr<float>(i)[j];
					counter++;
//					cout<<"aa"<<_o_balanced_depth_image->ptr<float>(i)[j]<<endl;
				}
				else
				{
					_o_balanced_depth_image->ptr<float>(i)[j]=0;
				}
			}//j
		}//i
//		cout <<"counter="<<counter<<",aver="<<sum/counter<<endl<<endl;
		return 1;
	}//void Get_balanced_depth_image(cv::Mat _original_depth_image, cv::Mat _x, cv::Mat _y, cv::Mat _z,	float _a1, float _a2, float _a3, float _a4,	cv::Mat* _o_balanced_depth_image);


};//class PlaneDetect




int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	PlaneDetect pd;
	ros::spin();
	return 0;
}//main
