/*********************************
 * ACCR_Comm: Serial Communication Node for ACCR UGTV
 * Dependencies: Serial_main
 * Protocol version: 0.1 (Aug, 2022)
 * More details can be found in README.md file in this package
 * *******************************/
 
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "decision/Bucket_pose.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

// Buffer size
#define	BUFFERSIZE 8

// Send/Receive Header
#define S_HEADER 0xAA
#define R_HEADER 0x55

// Setup Header
#define SHAKE_HEADER 0x00
#define INIT_HEADER 0x01

// Chassis Header
#define CHASSIS_VEL_HEADER 0x10
#define CHASSIS_POS_HEADER 0x11
#define CHASSIS_ORI_HEADER 0x12
#define CHASSIS_TAU_HEADER 0x13

// Bucket Header
#define BUCKET_MODE_HEADER 0x20
#define BUCKET_DYN_HEADER 0x21

// Utility Header
#define UTIL_MODE_HEADER 0x30

/* STOP HERE */

unsigned char s_chassis_buffer[S_CHASSIS_BUFFERSIZE];//send chassis buffer 
unsigned char s_util_buffer[S_UTIL_BUFFERSIZE];//send utility buffer 
unsigned char s_bucket_buffer[S_BUCKET_BUFFERSIZE];//send bucket buffer 

unsigned char r_buffer[R_BUFFERSIZE];//receive buffer
unsigned char r_chassis_buffer[R_CHASSIS_BUFFERSIZE];
unsigned char r_util_buffer[R_UTIL_BUFFERSIZE];
unsigned char r_bucket_buffer[R_BUCKET_BUFFERSIZE];



//union for fast conversion between float and hex
typedef union{
	unsigned char cvalue[4];
	int ivalue;
}int32_union;

typedef union{
	unsigned char cvalue[2];
	short int sivalue;
}int16_union;

typedef union{
	unsigned char cvalue[2];
	unsigned short int usivalue;
}uint16_union;

serial::Serial ser;



// Send buffer ---
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel){
	//ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	//ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;
	int16_t   vel_x = static_cast<int16_t>(cmd_vel.linear.x*1000);
	int16_t   ang_v = static_cast<int16_t>(cmd_vel.angular.z*1000);
	//ROS_INFO("linear vel: x-[%d]",vel_x);
	//ROS_INFO("angular vel: [%d]",ang_v);
	memset(s_chassis_buffer,CHASSIS_HEADER,sizeof(s_chassis_buffer));
	//linear ref speed [mm/s]
	s_chassis_buffer[1] = vel_x>>8;
	s_chassis_buffer[2] = vel_x;
	//anguar speed [mrad/s]
	s_chassis_buffer[3] = ang_v>>8;
	s_chassis_buffer[4] = ang_v;

	s_chassis_buffer[5] = 0;
	s_chassis_buffer[6] = 0;
	s_chassis_buffer[7] = 0;
	
	// for(int i=0;i<15;i++){
	// 	ROS_INFO("0x%02x",s_chassis_buffer[i]);
	// }
	
	ser.write(s_chassis_buffer,S_CHASSIS_BUFFERSIZE);
}

void bucket_msg_callback(const decision::Bucket_pose& bucket_msg){
	//int16_t   bucket_joint = bucket_msg.bucket_joint_angle.data;
	//ROS_INFO("I heard bucket: x-[%d],",bucket_msg.bucket_joint_angle.data);
	//bucket angle [mrad]
	memset(s_bucket_buffer,BUCKET_HEADER,sizeof(s_bucket_buffer));
	s_bucket_buffer[1] = bucket_msg.bucket_joint_angle.data>>8;
	s_bucket_buffer[2] = bucket_msg.bucket_joint_angle.data;
	//bucket speed [mrad/s]
	s_bucket_buffer[3] = bucket_msg.bucket_joint_speed.data>>8;
	s_bucket_buffer[4] = bucket_msg.bucket_joint_speed.data;
	s_bucket_buffer[5] = 0;
	s_bucket_buffer[6] = 0;
	s_bucket_buffer[7] = 0;

	for(int i = 0; i<3; i++)
	{
		ser.write(s_bucket_buffer, S_BUCKET_BUFFERSIZE);
		ros::Duration(0.05).sleep();
	}
}

void util_msg_callback()
{
	
}
// Send buffer ---

//Receive and check data
unsigned char data_analysis(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	//int i;
	if((buffer[0]==0xaa) && (buffer[1]==0xaa)){
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7]^
				buffer[8]^buffer[9]^buffer[10]^buffer[11]^buffer[12]^buffer[13]^
				buffer[14]^buffer[15]^buffer[16]^buffer[17]^buffer[18]^buffer[19]^
				buffer[20]^buffer[21]^buffer[22]^buffer[23]^buffer[24]^buffer[25];
		//ROS_INFO("check sum:0x%02x",csum);
		if(csum == buffer[26]){
			ret = 1;//Pass
		}
		else 
		  ret =0;//Fail, the data will not be processed.
	}
	/*
	for(i=0;i<rBUFFERSIZE;i++)
	  ROS_INFO("0x%02x",buffer[i]);
	*/
	return ret;

}



int main (int argc, char** argv){
    ros::init(argc, argv, "my_serial_node");
    ros::NodeHandle nh;
    
	
	
	//publish ROS topics
	ros::Publisher read_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);
	ros::Publisher bucket_pub = nh.advertise<decision::Bucket_pose>("bucket_msgs", 1000);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
    
	
	//subscribe ROS topics
	ros::Subscriber write_sub1 = nh.subscribe("/bucket_msgs",1000,bucket_msg_callback);
	ros::Subscriber write_sub = nh.subscribe("/cmd_vel",1000,cmd_vel_callback);
	
	//Define tf
	static tf::TransformBroadcaster odom_broadcaster;
	//Define data structure
	geometry_msgs::TransformStamped odom_trans;
	//Define object
	nav_msgs::Odometry odom;
	//Define quaternion variable
	geometry_msgs::Quaternion odom_quat;
	int32_union posx,posy; // position
	int16_union vx, va; // linear and angular velocity
	int32_union yaw; // yaw
	// bucket message
	decision::Bucket_pose bucket_msg;
	int16_union theta, theta_d, load; // bucket position, velocity, load
	//define time
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    //Communication frequency
    ros::Rate loop_rate(50);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
			ser.read(r_buffer,R_BUFFERSIZE);
			ROS_INFO("Stm32 info received");
			int i;
			for(i=0;i<R_BUFFERSIZE;i++)
				ROS_INFO("[0x%02x]",r_buffer[i]);
			ROS_INFO_STREAM("End reading from serial port");
			
			// if(data_analysis(r_buffer) != 0){
				if(r_buffer[0] == CHASSIS_HEADER){
					ROS_INFO("chassis buffer");
					int i;
					for(i=0;i<4;i++){
						posx.cvalue[i] = r_buffer[8-i];
						posy.cvalue[i] = r_buffer[12-i]; 
						yaw.cvalue[i] = r_buffer[16-i];
					}
					for(i=0;i<2;i++){
						vx.cvalue[i] = r_buffer[2-i];
						va.cvalue[i] = r_buffer[4-i];
						
					}
					//Convert the Euler angles to a quaternion
					odom_quat = tf::createQuaternionMsgFromYaw((double)yaw.ivalue/1000);
					
					odom_trans.header.frame_id = "odom";
					odom_trans.child_frame_id = "base_link";
					current_time = ros::Time::now();
					odom_trans.header.stamp = current_time;
					//Assign data
					odom_trans.transform.translation.x = (double)posx.ivalue/1000;//x
					odom_trans.transform.translation.y = (double)posy.ivalue/1000;//y
					odom_trans.transform.translation.z = 0;//z				
					odom_trans.transform.rotation = odom_quat;//q
					//tf transform
					odom_broadcaster.sendTransform(odom_trans);
					//Get ROS time
					current_time = ros::Time::now();
					//odom timestamp
					odom.header.stamp = current_time;
					
					odom.header.frame_id = "odom";
					odom.child_frame_id = "base_link";
					//
					odom.pose.pose.position.x = (double)posx.ivalue/1000;
					odom.pose.pose.position.y = (double)posy.ivalue/1000;
					odom.pose.pose.position.z = 0;
					odom.pose.pose.orientation = odom_quat;
					//
					odom.twist.twist.linear.x = (double)vx.sivalue/1000;
					// odom.twist.twist.linear.y = vy.fvalue;
					odom.twist.twist.angular.z = (double)va.sivalue/1000;
					//
					read_pub.publish(odom);
					ROS_INFO("publish odometry");
					last_time = current_time;		
				}else if(r_buffer[0] == BUCKET_HEADER){
					int i;
					for (i=0;i<2;i++){
						theta.cvalue[i] = r_buffer[1+i];
						theta_d.cvalue[i] = r_buffer[3+i];
						load.cvalue[i] = r_buffer[5+i];
					}
					bucket_msg.bucket_joint_angle.data = theta.sivalue;
					bucket_msg.bucket_joint_speed.data = theta_d.sivalue;
					bucket_pub.publish(bucket_msg);
				}else if(r_buffer[0] == UTIL_HEADER){

				}else {
					ROS_WARN("Invalid data header");
				}
			// }
			memset(r_buffer,0,R_BUFFERSIZE);
        }
        loop_rate.sleep();

    }
}

