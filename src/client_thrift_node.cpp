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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

#include <client_thrift/ImageRead.hpp>
#include "cJSON.h"
#include <chrono>

#include <thread>

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
		transport->close();
	};

	void receiveClientCb( const nav_msgs::OdometryConstPtr &odom );
	int getPosefromJson( const char *s );

	void get_img_thread();
	void exeCb();
public:
	ros::Subscriber odom_sub_;
	ros::Publisher reloc_pose_pub_;
	geometry_msgs::Pose2D mresult_reloc;

	Mat img;
	double odom_x_, odom_y_, odom_th_;

	shared_ptr<TTransport> socket;
	shared_ptr<TTransport> transport;
	shared_ptr<TProtocol> protocol;
	cv::VideoCapture video_cap;

private:
	void spin( const ros::TimerEvent& e);
	ros::Timer timer_;
	double freq;

};

SubOdomImgPubPose::SubOdomImgPubPose():socket( new TSocket("localhost", 9080) ),
                                       transport( new TBufferedTransport(socket) ),
                                       protocol( new TBinaryProtocol(transport) )
{
	std::cout << "sub pub in --" << std::endl;
	client = WithReturnServiceClient(protocol);
	freq = 25.0;
	transport->open();
	video_cap.open(0);
}

void SubOdomImgPubPose::exeCb()
{
	std::cout << "exeCb in --" << std::endl;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private("~");

	imageRead.init();

	odom_sub_ = nh_.subscribe( "/odom", 10, &SubOdomImgPubPose::receiveClientCb, this);
	reloc_pose_pub_ = nh_.advertise<geometry_msgs::Pose2D>( "/Q_reloc_pose2d", 10 );

	timer_ = nh_private.createTimer( ros::Duration(1.0/freq), &SubOdomImgPubPose::spin, this );

}

void SubOdomImgPubPose::spin(const ros::TimerEvent &e)
{
	std::cout << "runing ... " << std::endl;

	std::string result_pose;  //服务器返回值\

	/**
	 **@brief 缺少时间对齐
	 **/

	video_cap >> img;

	if( !img.empty() )
	{
		std::cout << "read img ok ... " << std::endl;

		///img imencode
		cv::resize( img, img, cv::Size(640,480) );
		std::vector<uchar> buf_img;
		cv::imencode( ".jpg", img, buf_img );
		std::string str_encode( buf_img.begin(), buf_img.end() );
		img.release();
		client.resultReturn( result_pose, str_encode, odom_x_, odom_y_, odom_th_ );

		if( getPosefromJson( result_pose.c_str() ) != 0 )
		{
			std::cerr << "格式转换失败!！  " << std::endl;
		}
		else
		{
			reloc_pose_pub_.publish( mresult_reloc );
		}
	}
	else
	{
		std::cout << "no img come!!! " << std::endl;
	}
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


	std::chrono::steady_clock::time_point time_start;
	std::chrono::steady_clock::time_point time_end;
	std::chrono::duration<double> time_span;
	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);
	std::cout << "run tim is:  " << time_span.count() << std::endl;

}

void SubOdomImgPubPose::get_img_thread()
{
	std::chrono::steady_clock::time_point time_start;
	std::chrono::steady_clock::time_point time_end;
	std::chrono::duration<double> time_span;
	video_cap.open(0);

	while (1)
	{
		time_start = std::chrono::steady_clock::now();

//		std::cout << "img in ... " << std::endl;

		img.release();

		//test
		video_cap >> img;
		time_end = std::chrono::steady_clock::now();

		time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);
//		std::cout << "run time is:  " << time_span.count() << std::endl;
	}
	//video_cap >> img;
//		imageRead.read_data( &img );
}

int main(int argc, char** argv)
{
	ros::init( argc, argv, "client_thrift_node" );
	ros::NodeHandle nh_;

	SubOdomImgPubPose subOdomImgPubPose;
//	std::thread getImgThread( &SubOdomImgPubPose::get_img_thread, &subOdomImgPubPose );
//	getImgThread.detach();
	subOdomImgPubPose.exeCb();

	ros::spin();
	return 0;
}

