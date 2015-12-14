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
static const int CONCERNED_RANGE=50;
static const int FINDING_RANGE=CONCERNED_RANGE+50;
static const int NUM_OF_INTERVAL_X=45;
static const int NUM_OF_INTERVAL_Y=38;
static const int NUM_OF_INTERVAL_TO_FIND_BOARD=NUM_OF_INTERVAL_X*NUM_OF_INTERVAL_Y;


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
		cv::namedWindow(WINDOW4);
	}
	~PlaneDetect()
	{
		cv::destroyWindow(WINDOW1);
		cv::destroyWindow(WINDOW2);
		cv::destroyWindow(WINDOW3);
		cv::destroyWindow(WINDOW4);
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

				cv::imshow(WINDOW2, cv_rgbptr->image);
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

/*

		int concerned_range=CONCERNED_RANGE;
		cv::Mat image_original(ROW_NUM,COL_NUM,CV_8UC3);
		cv::Mat image_balanced(ROW_NUM,COL_NUM,CV_32FC1);
		image_original.zeros(ROW_NUM,COL_NUM,CV_32FC1);
		cv::Mat image_derivative(ROW_NUM,COL_NUM,CV_32FC1);


		int i=240;
		float z_sum=0;
		int z_counter=0;

		for (int j=concerned_range;j<COL_NUM-concerned_range;j++)
		{
			if ((Zi[j]>0)&&(Zi[j]<5000))
			{
				int z=Zi[j];
				cv::circle(image_original, cv::Point(j,z%480),1,cv::Scalar(0,0,255));
				z_sum+=Zi[j];
				z_counter++;
			}
			if((Zi[j+concerned_range]>0)&&(Zi[j+concerned_range]<5000)
					&&(Zi[j-concerned_range]>0)&&(Zi[j-concerned_range]<5000))
				Di[j]=(Zi[j+concerned_range]-Zi[j-concerned_range])/(2*concerned_range);
			else
				Di[j]=5000;

		}


		float z_aver=z_sum/z_counter;
		int d_counter=0;

		float slope_sqr_sum=0;
		for (int j=concerned_range;j<COL_NUM-concerned_range;j++)
		{
			if((Di[j]>-5000)&&(Di[j]<5000))
			{
				d_counter++;
				slope_sqr_sum+=Di[j]*Di[j];

			}
		}


		float slope_aver=sqrt(slope_sqr_sum/d_counter);
		printf("a%d,b%f,c%f",d_counter,slope_sqr_sum,slope_aver);
		float tolerance=2*fabs(slope_aver);

		int new_dcounter=0;
		float new_slope_sum=0;
		for (int j=concerned_range;j<COL_NUM-concerned_range;j++)
		{
			if(fabs(Di[j])<tolerance)
			{
				new_dcounter++;
				new_slope_sum+=Di[j];
//				printf("dddd%f",Di[j]*Di[j]);
			}
		}

		float new_slope_aver=new_slope_sum/new_dcounter;
		printf("d%d,e%f,f%f\n",new_dcounter,new_slope_aver,new_slope_sum);

		for (int j=concerned_range;j<COL_NUM-concerned_range;j++)
		{
			int z=Zi[j]-new_slope_aver*(j-320);
			int z_int_aver=z_aver;
			cv::circle(image_origina, cv::Point(j,z%480),1,cv::Scalar(0,255,0));
			cv::circle(image_original, cv::Point(j,z_int_aver%480),1,cv::Scalar(255,0,0));
		}
		/*
		float kxsqr_sum=0;
		int counterx=0;
		int counter2x=0;
		int concerned_range=10;
		cv::Mat image_derivative_x(ROW_NUM,COL_NUM,CV_32FC1);
		cv::Mat image_derivative2_x(ROW_NUM,COL_NUM,CV_32FC1);
		float derivative2_xtol=2;

		//y
		float ky_sum=0;
		int countery=0;
		int kx_plus_counter=1;
		int kx_minus_counter=1;
		float sum_sqr_kx_plus=0;
		float sum_sqr_kx_minus=0;
		cv::Mat image_derivative_y(ROW_NUM,COL_NUM,CV_32FC1);
		for(int i=concerned_range;i<ROW_NUM-concerned_range;i++)
		{

			for (int j=concerned_range;j<COL_NUM-concerned_range;j++)
			{
				image_derivative_x.ptr<float>(i)[j]=
					(_cvimageptr->image.ptr<float>(i)[j+1]
					-_cvimageptr->image.ptr<float>(i)[j])/(2*concerned_range);

				counterx++;



				float t_derivative2=fabs(_cvimageptr->image.ptr<float>(i)[j-1]
								+_cvimageptr->image.ptr<float>(i)[j+1]
								-2*_cvimageptr->image.ptr<float>(i)[j]);
				if((t_derivative2<derivative2_xtol))
				{
					image_derivative2_x.ptr<float>(i)[j]=255;
					counter2x++;

					if(image_derivative_x.ptr<float>(i)[j]>0)
					{
						kx_plus_counter++;
						sum_sqr_kx_plus+=image_derivative_x.ptr<float>(i)[j]*image_derivative_x.ptr<float>(i)[j];
					}
					if(image_derivative_x.ptr<float>(i)[j]<0)
					{
						kx_minus_counter++;
						sum_sqr_kx_minus+=image_derivative_x.ptr<float>(i)[j]*image_derivative_x.ptr<float>(i)[j];

					}

				}
				else
				{
					image_derivative2_x.ptr<float>(i)[j]=0;
				}


				//y direction
				if((_cvimageptr->image.ptr<float>(i+1)[j]!=0)&&(_cvimageptr->image.ptr<float>(i)[j]!=0))
				{
					image_derivative_y.ptr<float>(i)[j]=
							_cvimageptr->image.ptr<float>(i+1)[j]-_cvimageptr->image.ptr<float>(i)[j];
					ky_sum+=image_derivative_y.ptr<float>(i)[j];
					countery++;
				}

			}
		}

		float t_ratio=kx_plus_counter/kx_minus_counter;
		float kx_aver;
		if(t_ratio>1.1)
		{
			kx_aver=sqrt(sum_sqr_kx_plus/kx_plus_counter);

		}
		else if(t_ratio<1/1.1)
		{
			kx_aver=-sqrt(sum_sqr_kx_minus/kx_minus_counter);
		}
		else
		{
			kx_aver=0;
		}

		cv::Mat image_balanced_x(ROW_NUM,COL_NUM,CV_32FC1);
		float depth_sum=0;
		float depth_balanced_sum=0;
		float depth_cov=0;
		float depth_balanced_cov=0;
		int counter_cov;
		for(int i=concerned_range;i<ROW_NUM-concerned_range;i++)
		{
			for (int j=concerned_range;j<COL_NUM-concerned_range;j++)
			{
				if(image_derivative2_x.ptr<float>(i)[j]!=0)
				{
					image_balanced_x.ptr<float>(i)[j]=_cvimageptr->image.ptr<float>(i)[j+1]+kx_aver*(j-COL_NUM/2);					depth_sum+=_cvimageptr->image.ptr<float>(i)[j];
					depth_balanced_sum+=image_balanced_x.ptr<float>(i)[j];
				}
			}
		}

		float depth_aver=depth_sum/counterx;
		float depth_balanced_aver=depth_balanced_sum/counterx;
		for(int i=concerned_range;i<ROW_NUM-concerned_range;i++)
		{
			for (int j=concerned_range;j<COL_NUM-concerned_range;j++)
			{
				if(image_derivative2_x.ptr<float>(i)[j]!=0)
				{
					depth_cov+=(_cvimageptr->image.ptr<float>(i)[j]-depth_aver)*(_cvimageptr->image.ptr<float>(i)[j]-depth_aver);
					depth_balanced_cov+=(image_balanced_x.ptr<float>(i)[j]-depth_balanced_aver)
								*(image_balanced_x.ptr<float>(i)[j]-depth_balanced_aver);

				}

			}
		}
		depth_cov=depth_cov/counterx;
		depth_balanced_cov=depth_balanced_cov/counterx;


	//	float kx_aver=kx_sum/counterx;
		float ky_aver=ky_sum/countery;
		ROS_INFO("%d,%d,%d,%d,%f",counterx,counter2x,kx_plus_counter,kx_minus_counter,kx_aver);
		ROS_INFO("%f,%f,%f,%f",depth_aver,depth_balanced_aver,depth_cov,depth_balanced_cov);

		int counterx_board=0;
		float k_sum_board=0;
		float tolerencex;
	//	if (kx_aver<1) tolerencex=5+5*fabs(kx_aver);
	//	else tolerencex=10*fabs(kx_aver);


/*		for(int i=0;i<ROW_NUM;i++)
		{
			for (int j=0;j<COL_NUM;j++)
			{

				if((fabs(image_derivative_x.ptr<float>(i)[j])<tolerence)&&(image_derivative_x.ptr<float>(i)[j]!=0))
				{
					k_sum_board+=image_derivative_x.ptr<float>(i)[j];
					counter_board++;
					float density=image_derivative_x.ptr<float>(i)[j]/tolerence;
					cv::circle(image_board,cv::Point(j,i),0,cv::Scalar(0,0,255*density));
				}
		}
		}

*/
		//				image_balanced_x.ptr<float>(i)[j]=_cvimageptr->image.ptr<float>(i)[j]-j*k_aver;

//		float k_aver_board=k_sum_board/counter_board;

//		float percent=(float)counter_board/640/480*100;
//		ROS_INFO("%d,%f,%f,%f",counter,k_min,k_aver,k_max);
//		ROS_INFO("      %f,%d,%.1f,%.3f",tolerence,counter_board,percent,k_aver_board);


		//convert depth pixel to xyz
		cv::Mat matx(ROW_NUM,COL_NUM,CV_32FC1);
		cv::Mat maty(ROW_NUM,COL_NUM,CV_32FC1);
		cv::Mat matz(ROW_NUM,COL_NUM,CV_32FC1);

		Convert_depth_image_to_xyz_data(_cvimageptr->image, &matx,&maty,&matz);

		cv::Mat image_color(ROW_NUM,COL_NUM,CV_8UC3);
		Test_line_derivative(240,matx,matz,&image_color);

		cv::Mat balanced_image(ROW_NUM,COL_NUM,CV_32FC1);
//		float z_aver[ROW_NUM];
		Get_balanced_image(matx,matz,&balanced_image);
		cv::Mat object_image(ROW_NUM,COL_NUM,CV_8UC3);
		cv::Mat object_number(ROW_NUM,COL_NUM,CV_32FC1);

		Display_object(balanced_image,&object_image,&object_number);
/*
		float a1=0;
		float a2=0;
		float a3=0;
		float a4=0;


		Find_board(matx, maty, matz, &a1,&a2,&a3,&a4);
		cout << "a1=" << a1 << ",a2=" << a2 << ",a3=" << a3 << ",a4=" << a4 << endl;


		cv::Mat image_balanced_depth_image(ROW_NUM,COL_NUM,CV_32FC1);
		Get_balanced_depth_image(_cvimageptr->image,matx,maty,matz,a1,a2,a3,a4,
								&image_balanced_depth_image);

		cv::Mat image_colored_depth(ROW_NUM,COL_NUM,CV_8UC3);
		for(int i=CONCERNED_RANGE; i<ROW_NUM-CONCERNED_RANGE;i++)
		{
			for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
			{
				int r=0;
				int g=0;
								int b=0;
				if(image_balanced_depth_image.ptr<float>(i)[j]<0)
				{
					r=255;
				}
				else if(image_balanced_depth_image.ptr<float>(i)[j]==0)
				{
					r=0;
				}
				else
				{
					r=128;
				}
				cv::circle(image_colored_depth, cv::Point(j,i),1,cv::Scalar(b,g,r));

			}
		}

*/

		cv::imshow(WINDOW2, _cvimageptr->image);
		cv::imshow(WINDOW1, image_color);
//		cv::imshow(WINDOW3, object_image);
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
	//	float Di[COL_NUM];
		float Bi[COL_NUM];
		float sum_z=0;
		int counter=1;
		float min_z=10000;
		float max_z=-10000;
		for (int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{
			if(Zi[j]>0)
			{
				sum_z+=Zi[j];
				counter++;
				if(Zi[j]>max_z) max_z=Zi[j];
				if(Zi[j]<min_z) min_z=Zi[j];
				int z=(int)Zi[j];
				cv::circle(*_o_color_image, cv::Point(j,z%480),1,cv::Scalar(0,0,255));
			}
		}//for
		float aver_z=sum_z/counter;
//		cout <<"min"<<min_z<<"max"<<max_z<<"aver"<<aver_z<<endl;
		int t_z=(int)(aver_z);
		cv::line(*_o_color_image, cv::Point(0,t_z%480),cv::Point(639,t_z%480),cv::Scalar(0,255,0),1);

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
		//matrix_X.col(0)<<Xi[100],Zi[100],1;
		//matrix_X.col(1)<<Xi[450],Zi[450],1;
		matrix_L=matrix_X*matrix_X.transpose();
		Eigen::EigenSolver<Eigen::MatrixXf> es(matrix_L);

		float hmin=0;
		int hindex=0;
		for (int i=0;i<3;i++)
		{
			std::complex<float> lambda=es.eigenvalues()[i];
			if(lambda.imag()==0)
			{
				if(hmin==0) {hmin=lambda.real();hindex=i;}
				else
				{
					if(lambda.real()<hmin)	{hmin=lambda.real();hindex=i;}
				}
			}
		}


		//output
		if(hmin==0) {cout << "error" << endl;}

		Eigen::VectorXcf hvector=es.eigenvectors().col(hindex);
		Eigen::VectorXf hvector_real=hvector.real();
		float k,b;
		k=-hvector_real[0]/hvector_real[1];
		b=-hvector_real[2]/hvector_real[1];

		int kb_z1=k*(-320)+b;
		int kb_z2=k*(639-320)+b;
		cv::line(*_o_color_image, cv::Point(0,kb_z1%480),cv::Point(639,kb_z2%480),cv::Scalar(255,0,0),1);


		for(int j=CONCERNED_RANGE;j<COL_NUM-CONCERNED_RANGE;j++)
		{
			Bi[j]=Zi[j]-k*Xi[j]-b;
			int z=Bi[j]+aver_z;
			cv::circle(*_o_color_image, cv::Point(j,z%480),1,cv::Scalar(255,0,255));

		}

//		cout<<"lambda"<<hmin<<"k"<<k<<"b"<<b<<endl;


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
		cout<<"depth from sensor to plane:"<<b_aver<<endl;

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
		cout<<"depth of object above plane:"<<object_d_aver<<endl;

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
		int max_object=3;
		if (set_object_index<3)
		{
			max_object=set_object_index;
		}
		for(int k=0;k<max_object;k++)
		{
			cout<<"order:"<<object_order[k]<<",size"<<object_counter[object_order[k]]<<endl;
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
				int g=k*100;
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

//			cout<<"minx,miny"<<min_x<<","<<min_y<<",maxx,maxy"<<max_x<<","<<max_y<<endl;

			cv::Point point=cv::Point(mid_x,mid_y);
			cv::putText(*_o_object_image,str.str(),point ,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255),3);
			cv::line(*_o_object_image,cv::Point(mid_x-15,mid_y),cv::Point(mid_x+15,mid_y),cv::Scalar(255,255,0),2);
			cv::line(*_o_object_image,cv::Point(mid_x,mid_y-15),cv::Point(mid_x,mid_y+15),cv::Scalar(255,255,0),2);
			cv::rectangle(*_o_object_image,cv::Point(min_x,min_y),cv::Point(max_x,max_y),cv::Scalar(255,255,0),1);



		}//k  number of object displayed
						cv::imshow(WINDOW3, *_o_object_image);
						cv::waitKey(3);

	}//function


	void Find_board(cv::Mat _i_x, cv::Mat _i_y, cv::Mat _i_z,
			float* _o_a1, float* _o_a2, float* _o_a3, float* _o_a4)
	{
		//plane equation: a1*x+a2*y+a3*z+a4=0
		//we need to calculate a1,a2,a3,a4
		//let A=[a1,a2,a3,a4]T, Xn=[x,y,z,1]T, X=[X1,X2,X3]
		//AT*X=0, X1,X2,X3 is given, we can calculate A, notice |A|=1

		//if the NUM of input points (size of X) is larger than 3, X=[X1,X2,X3,X4,X5,...]
		//assume every points is on AT*X=0
		//to get the best A, we need the value of sum(|AT*Xn|) minimized
		//sum(|AT*Xn|)=|AT*X|=sqrt(|(AT*X)*(AT*XT)T|)=sqrt(|AT*X*XT*A|)=sqrt(|AT*(X*XT)*A|)
		//let L=X*XT
		//the minimum e-value is |h|, corresponding e-vector is p
		//L*p=h*p
		//|pT*L*p|=|pT*h*p|=|h*pT*p|=|h|
		//min sqrt |h| is the min value of |AT*X|
		//this value is the total distance from the points to the plane
		//notice |h|min<=|xTAx|<=|h|max
		//xTAx=xT*PT*U*P*x=zTUz, U is diagnol, max=|h|max, min=|h|min

		Eigen::MatrixXf matrix_A(4,1);
		Eigen::MatrixXf matrix_X(4,NUM_OF_INTERVAL_TO_FIND_BOARD);
		Eigen::MatrixXf matrix_L(4,4);

		int start_point_i=FINDING_RANGE;
		int end_point_i=ROW_NUM-FINDING_RANGE;
		int start_point_j=FINDING_RANGE;
		int end_point_j=COL_NUM-FINDING_RANGE;
		int interval_i=(int)(floor(end_point_i-start_point_i)/NUM_OF_INTERVAL_Y);
		int interval_j=(int)(floor(end_point_j-start_point_j)/NUM_OF_INTERVAL_X);

		int counter=1;
		for(int i=0;i<NUM_OF_INTERVAL_X;i++)
		{
			for(int j=0;j<NUM_OF_INTERVAL_Y;j++)
			{
				int ii=start_point_i+i*interval_i+int(floor(interval_i/2));
				int jj=start_point_j+j*interval_j+int(floor(interval_j/2));
				float x=_i_x.ptr(ii)[jj];
				float y=_i_y.ptr(ii)[jj];
				float z=_i_z.ptr(ii)[jj];
				matrix_X.col(counter)<< x,y,z,1;
				counter++;
			}//j
		}//i

		matrix_L=matrix_X*matrix_X.transpose();
		Eigen::EigenSolver<Eigen::MatrixXf> es(matrix_L);

		float hmin=0;
		int hindex=0;
		for (int i=0;i<4;i++)
		{
			std::complex<float> lambda=es.eigenvalues()[i];
			if(lambda.imag()==0)
			{
				if(hmin==0) {hmin=lambda.real();hindex=i;}
				else
				{
					if(lambda.real()<hmin)	{hmin=lambda.real();hindex=i;}
				}
			}
		}


		//output
//		if(hmin==0) {cout << "error" << endl;}

		Eigen::VectorXcf hvector=es.eigenvectors().col(hindex);
		Eigen::VectorXf hvector_real=hvector.real();
		*_o_a1=hvector_real[0];
		*_o_a2=hvector_real[1];
		*_o_a3=hvector_real[2];
		*_o_a4=hvector_real[3];

	}//void Find_board(cv::Mat _i_x, cv::Mat _i_y, cv::Mat _i_z, float* _o_a1, float* _o_a2, float* _o_a3, float* _o_a4)

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
