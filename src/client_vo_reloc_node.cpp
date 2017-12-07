//
// Created by bpeer on 17-9-18.
//
#include "ros/ros.h"

#include <stdio.h>
#include <string>
#include "transport/TSocket.h"
#include "protocol/TBinaryProtocol.h"
#include "server/TSimpleServer.h"
#include "transport/TServerSocket.h"
#include "transport/TBufferTransports.h"
#include "withReturn_types.h"
#include "WithReturnService.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

#include <client_thrift/ImageRead.hpp>
#include "cJSON.h"
#include <chrono>

#include <tf/transform_listener.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;
using boost::shared_ptr;

static ImageRead imageRead; //内存读取数据

WithReturnServiceClient client;  //add 构造函数


class SubOdomImgPubPose
{
public:
	SubOdomImgPubPose();
	~SubOdomImgPubPose()
	{
		fs.close();
		transport->close();
	};

	void receiveClientCb( const nav_msgs::OdometryConstPtr &odom );
	int getPosefromJson( const char *s );

	void exeCb();
public:
	ros::Subscriber odom_sub_;
	ros::Publisher reloc_pose_pub_;
	ros::Publisher reloc_motion_pub_;
	geometry_msgs::Pose2D mresult_reloc;
	std_msgs::Bool breloc_motion;

	tf::StampedTransform transform_;
	tf::TransformListener tf_listener_;

	Mat img;
	double odom_x_, odom_y_, odom_th_;

	shared_ptr<TTransport> socket;
	shared_ptr<TTransport> transport;
	shared_ptr<TProtocol> protocol;
	cv::VideoCapture video_cap;

	std::ofstream fs;

	int test_num_; //@todo test

private:
	void spin( const ros::TimerEvent& e);
	ros::Timer timer_;
	double freq;

	Data data;
};

SubOdomImgPubPose::SubOdomImgPubPose():socket( new TSocket("192.168.1.168", 10086) ),
                                       transport( new TBufferedTransport(socket) ),
                                       protocol( new TBinaryProtocol(transport) )
{
	std::cout << "sub pub in --" << std::endl;
	client = WithReturnServiceClient(protocol);
	freq = 30.0;
	transport->open();
	test_num_ = 0; //@todo test
	fs.open("testData.txt", std::ios::app);

	//test
	//video_cap.open(0);
}

void SubOdomImgPubPose::exeCb()
{
	std::cout << "exeCb in --" << std::endl;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private("~");

	imageRead.init();

	odom_sub_ = nh_.subscribe( "/odom", 10, &SubOdomImgPubPose::receiveClientCb, this);
	reloc_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>( "/Q_reloc_pose", 10 );
	reloc_motion_pub_ = nh_.advertise<std_msgs::Bool>( "/Q_pose_status", 2 );
	timer_ = nh_private.createTimer( ros::Duration(1.0/freq), &SubOdomImgPubPose::spin, this );

}

void SubOdomImgPubPose::spin(const ros::TimerEvent &e)
{
	std::cout << "runing ... " << std::endl;
	test_num_++;  //@todo test
//	for (int i = 0; i < 3 ; ++i) {

//	}
	std::cout << "pub motion ... " << std::endl;

	ros::Time  start_t_ = ros::Time::now();  //test time

	double tmp_x, tmp_y, tmp_th;  //for 时间对齐

	std::string result_pose;  //服务器返回值

	/**
	 **@brief 缺少时间对齐
	 **/
	//video_cap >> img;
//		imageRead.read_data( &img );  //decide empty
	int num_flag;
	bool img_flag = false;
	{
		while( !img_flag )
		{
//			video_cap >> img;
			imageRead.read_data( &img );
			if (!img.empty())
			{
				breloc_motion.data = 1;
				reloc_motion_pub_.publish( breloc_motion );
				img_flag = true;
			}
			else
			{
				++num_flag;
			}
			if(num_flag > 1000)
			{
				std::cout << "no img coming, please check it !!!" << std::endl;
//				sleep(1);
				return;  //@todo 这儿需要再优化处理一下
			}
		}
	}
	std::cout << "read img ok ... " << std::endl;


	data.x = odom_x_;
	data.y = odom_y_;
	data.th = odom_th_;

	///img imencode
	cv::resize( img, img, cv::Size(640,480) );
	std::vector<uchar> buf_img;
	cv::imencode( ".jpg", img, buf_img );
	std::string str_encode( buf_img.begin(), buf_img.end() );

	client.resultReturn( result_pose, str_encode, "myhid", data );

	std::cout << "result: " << result_pose << std::endl;

	/*
	if ( result_pose.empty() )
		std::cerr << " relocal failed..." << std::endl;  //@todo  这可以返回失败的情况
	else
	{
		//@todo 这个需要根据后续要求加入 odomX odomY odomTh;;
		if( getPosefromJson( result_pose.c_str() ) != 0 )
		{
			std::cerr << "格式转换失败!！  " << std::endl;
			mresult_reloc.x = .0;
			mresult_reloc.y = .0;
			mresult_reloc.theta = 10.0;
			reloc_pose_pub_.publish( mresult_reloc );
		}
		else
		{
			bool tf_flag = false;
			double x = .0, y = .0, th = .0;

			//get tf pose to test
			while( !tf_flag )
			{
				try
				{
					tf_listener_.lookupTransform( "/map", "/base_link", ros::Time(0), transform_ );
					x = transform_.getOrigin().getX();
					y = transform_.getOrigin().getY();
					th = tf::getYaw( transform_.getRotation() );

					tf_flag = true;
				}
				catch(tf::TransformException &ex)
				{
					tf_flag = false;
				}
			}

			ros::Time  end_t_ = ros::Time::now();

			//@todo  需要时间戳最好,,协商而定
			reloc_pose_pub_.publish( mresult_reloc );


			if( fs.is_open() )
			{
				fs << "test_num:" << test_num_ << " ";
				fs << " Time:" << end_t_
				   << " delta_time:" << end_t_ - start_t_;
				fs << " delta_x:" << x - mresult_reloc.x
				   << " delta_y:" << y - mresult_reloc.y
				   << " delta_theta:" << th - mresult_reloc.theta
				   << "   ";
				fs << " real_x:" << x
				   << " real_y:" << y
				   << " real_th:" << th
				   << "   ";
				fs << " reloc_x:" << mresult_reloc.x
				   << " reloc_y:" << mresult_reloc.y
				   << " reloc_th:" << mresult_reloc.theta << std::endl << std::endl;
			}

		}

	}  //end --result_pose.empty()
	*/

}


int SubOdomImgPubPose::getPosefromJson(const char *s)
{
	cJSON *root = cJSON_Parse( s );
	if( !root )
	{
		std::cout << "get root failed!" << std::endl;
		return -1;
	}

	//x
	cJSON *js_x = cJSON_GetObjectItem(root, "x");
	if( !js_x )
	{
		std::cout << "No x!" << std::endl;
		return -1;
	}
	mresult_reloc.x = js_x->valuedouble;

	//y
	cJSON *js_y = cJSON_GetObjectItem(root, "y");
	if( !js_y )
	{
		std::cout << "No y!" << std::endl;
		return -1;
	}
	mresult_reloc.y = js_y->valueint;

	//th
	cJSON *js_th = cJSON_GetObjectItem(root, "th");
	if( !js_th )
	{
		std::cout << "No th!" << std::endl;
		return -1;
	}
	mresult_reloc.theta = js_th->valueint;

	cJSON_Delete(root);
	return 0;
}

void SubOdomImgPubPose::receiveClientCb(const nav_msgs::OdometryConstPtr &odom)
{

	std::cout << "odom in ... " << std::endl;

	odom_x_ = odom->pose.pose.position.x;
	odom_y_ = odom->pose.pose.position.y;
	odom_th_ = tf::getYaw( odom->pose.pose.orientation );

//	std::chrono::steady_clock::time_point time_start;
//	std::chrono::steady_clock::time_point time_end;
//	std::chrono::duration<double> time_span;
//	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);
//	std::cout << "run tim is:  " << time_span.count() << std::endl;
}


int main(int argc, char** argv)
{
	ros::init( argc, argv, "client_thrift_node" );
	ros::NodeHandle nh_;

	SubOdomImgPubPose subOdomImgPubPose;

	subOdomImgPubPose.exeCb();

	ros::spin();
	return 0;
}

