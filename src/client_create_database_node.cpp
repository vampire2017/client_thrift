#include "CreateDbService.h"//
// Created by bpeer on 17-9-20.
//

#include "ros/ros.h"

#include <stdio.h>
#include <string>
#include "transport/TSocket.h"
#include "protocol/TBinaryProtocol.h"
#include "server/TSimpleServer.h"
#include "transport/TServerSocket.h"
#include "transport/TBufferTransports.h"
#include "createDb_types.h"
#include "CreateDbService.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <client_thrift/ImageRead.hpp>
#include <chrono>
#include "timer.h"

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;
using boost::shared_ptr;

static ImageRead imageRead; //内存读取数据

CreateDbServiceClient client;  //add 构造函数


class SubQposeImg
{
public:
	SubQposeImg();
	~SubQposeImg()
	{
		transport->close();
	};

	void receiveMap2BaseCb( const tf::tfMessage &map2base );

	void exeCb();
public:
	ros::Subscriber Qpose_sub_;

	Mat img;
	double pose_x_, pose_y_, pose_th_;
	int pose_stamp_;

	shared_ptr<TTransport> socket;
	shared_ptr<TTransport> transport;
	shared_ptr<TProtocol> protocol;
	cv::VideoCapture video_cap;

private:
	void spin( const ros::TimerEvent& e);
	ros::Timer timer_;
	int freq;
	internalTimer::Timer time_stamp;
};

SubQposeImg::SubQposeImg():socket( new TSocket("192.168.1.168", 10087) ),
                                       transport( new TBufferedTransport(socket) ),
                                       protocol( new TBinaryProtocol(transport) )
{
	std::cout << "sub pub in --" << std::endl;
	client = CreateDbServiceClient(protocol);
	freq = 30;
	transport->open();

	//for test
//	video_cap.open(0);
}

void SubQposeImg::exeCb()
{
	std::cout << "exeCb in --" << std::endl;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private("~");

	imageRead.init();

//	std::cout << "odom in ... " << std::endl;
//	odom_x_ = map2odom.transforms.begin()->transform.translation.x;
//	odom_y_ = map2odom.transforms.begin()->transform.translation.y;
//	odom_th_ = tf::getYaw( map2odom.transforms.begin()->transform.rotation );

//	Qpose_sub_ = nh_.subscribe( "/Q_pose", 10, &SubQposeImg::receiveMap2BaseCb, this);

	Qpose_sub_ = nh_.subscribe<tf::tfMessage>( "/Q_pose", 10,
	                            [this](const tf::tfMessageConstPtr &map2base){
		                        std::cout << "odom in ... " << std::endl;

		                        pose_x_ = map2base->transforms.begin()->transform.translation.x;
								pose_y_ = map2base->transforms.begin()->transform.translation.y;
								pose_th_ = tf::getYaw( map2base->transforms.begin()->transform.rotation );
		                        pose_stamp_ = time_stamp.ToDAY();
	                            } );

	timer_ = nh_private.createTimer( ros::Duration(1.0/freq), &SubQposeImg::spin, this );

}

void SubQposeImg::spin(const ros::TimerEvent &e)
{
	std::cout << "runing ... " << std::endl;
	/**
	 **@brief 缺少时间对齐
	 **/
	//video_cap >> img;
//		imageRead.read_data( &img );  //decide empty
	int num_flag = 0;
	bool img_flag = false;
	{
		while( !img_flag )
		{
			video_cap >> img;
			if (!img.empty())
				img_flag = true;
			else
			{
				++num_flag;
			}
			if(num_flag > 1000)
			{
				std::cout << "no img coming, please check it !!!" << std::endl;
				sleep(1);
				return;  //@todo 这儿需要再优化处理一下
			}
		}
	}
	std::cout << "read img ok ... " << std::endl;

	//@todo 尚未考虑..没有数据的时候odom_x_
	Data data;
	data.x = pose_x_;
	data.y = pose_y_;
	data.th = pose_th_;
	data.stamp = pose_stamp_;
	std::cout << "stamp:  " << pose_stamp_;

	///img imencode
	cv::resize( img, img, cv::Size(640,480) );
	std::vector<uchar> buf_img;
	cv::imencode( ".jpg", img, buf_img );
	std::string str_encode( buf_img.begin(), buf_img.end() );

	client.createDb( str_encode, data );
}


void SubQposeImg::receiveMap2BaseCb(const tf::tfMessage &map2odom)
{

	std::cout << "odom in ... " << std::endl;
	pose_x_ = map2odom.transforms.begin()->transform.translation.x;
	pose_y_ = map2odom.transforms.begin()->transform.translation.y;
	pose_th_ = tf::getYaw( map2odom.transforms.begin()->transform.rotation );

}


int main(int argc, char** argv)
{
	ros::init( argc, argv, "client_thrift_node" );
	ros::NodeHandle nh_;

	SubQposeImg subQposeImg;

	subQposeImg.exeCb();

	ros::spin();
	return 0;
}

//  //test
//	std::chrono::steady_clock::time_point time_start;
//	std::chrono::steady_clock::time_point time_end;
//	std::chrono::duration<double> time_span;
//	time_span = std::chrono::duration_cast<std::chrono::duration<double>>(time_end - time_start);
//	std::cout << "run tim is:  " << time_span.count() << std::endl;
