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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "accr_msgs/Bucket.h"
#include "accr_msgs/Chassis.h"
#include "accr_msgs/Utility.h"

// Buffer size
#define	BUFFERSIZE 10 

// Send/Receive Header
#define S_HEADER 0xAA
#define R_HEADER 0x55

// Setup Header
#define SHAKE_HEADER 0x00
#define INIT_HEADER 0x01

// Chassis Header
#define CHASSIS_VEL_HEADER 0x10 // chassis velocity
#define CHASSIS_POS_HEADER 0x11 // chassis position
#define CHASSIS_ORI_HEADER 0x12 // chassis orientation
#define CHASSIS_TAU_HEADER 0x13 // chassis torque

// Bucket Header
#define BUCKET_MODE_HEADER 0x20 // bucket mode
#define BUCKET_DYN_HEADER 0x21 // bucket dynamics

// Utility Header
#define UTIL_MODE_HEADER 0x30

// Error Header
#define ERROR_HEADER 0x01 

/* STOP HERE */

unsigned char s_chassis_buffer[BUFFERSIZE]; //send chassis buffer 
unsigned char s_util_buffer[BUFFERSIZE];    //send utility buffer 
unsigned char s_bucket_buffer[BUFFERSIZE];  //send bucket buffer 
unsigned char s_init_buffer[BUFFERSIZE];    //send initialization buffer

unsigned char r_buffer[BUFFERSIZE];         //receive buffer

//union for fast conversion between float and hex
typedef uint16_t u16;
typedef uint8_t u8;
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

typedef struct{
	u16 ts = 0.02;
	u8 freq_div = 50;
	u8 chassis_op = 0x0f;
	u8 bucket_op = 0x03;
	u8 other_op = 0x01;
	u8 reset_op = 1; 
}init_op_t;

serial::Serial ser; // Serial port object
ros::Subscriber bucket_sub;
ros::Subscriber cmd_vel_sub;
ros::Subscriber util_sub;
ros::Publisher odom_pub;
ros::Publisher bucket_pub; // publish bucket state feedback to ROS topic
ros::Publisher wheel_torque_pub;
ros::Publisher util_pub;
// ros::Publisher error_pub; // publish error message from micro-controller

geometry_msgs::TransformStamped odom_trans; //Define data structure
nav_msgs::Odometry odom; //Define object
geometry_msgs::Quaternion odom_quat; //Define quaternion variable
accr_msgs::Bucket bucket_msg; // bucket message
accr_msgs::Chassis wheel_torque; 
accr_msgs::Utility util_msgs;
ros::Time current_time, last_time;
init_op_t init_op; 
int32_union posx,posy; // position
int16_union vx, va; // linear and angular velocity
int16_union tau_l, tau_r;
int32_union yaw; // yaw
int16_union ext1, ext2, load; // arm extension length, forarme extension length, load
int16_union f1l, f2l, f1r, f2r; // left arm force, left forearm force, right arm force, right forearm force

int comm_init();
void mc_init();
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel);
void bucket_msg_callback(const accr_msgs::Bucket& bucket_cmd);
void util_msg_callback(const accr_msgs::Utility& util_cmd);




int main (int argc, char** argv)
{
    ros::init(argc, argv, "accr_comm");
	tf::TransformBroadcaster odom_broadcaster;

	comm_init();

    // Communication frequency
    ros::Rate loop_rate(10);
	int i;

	// Microcontroller-PC handshake
	memset(s_init_buffer, 0, BUFFERSIZE);
	memset(r_buffer, 0, BUFFERSIZE);
	s_init_buffer[0] = S_HEADER;
	s_init_buffer[1] = SHAKE_HEADER;

	while(ros::ok()){
		ros::spinOnce();

		ser.write(s_init_buffer, BUFFERSIZE);
		ROS_INFO("Connecting microcontroller... ");

		if(ser.available()){
			ser.read(r_buffer, BUFFERSIZE);
			if(r_buffer[0] == R_HEADER && r_buffer[1] == SHAKE_HEADER){
				mc_init();
				ROS_INFO("Microcontroller connected. ");
				break;
			}
		}
		ROS_WARN("Connection failed, retrying... ");
		
		loop_rate.sleep();
	}

    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
			ser.read(r_buffer,BUFFERSIZE);
			ROS_INFO("Stm32 info received");
			for(i=0;i<BUFFERSIZE;i++)
				ROS_INFO("[0x%02x]",r_buffer[i]);
			ROS_INFO_STREAM("End reading from serial port");
		}else {
			ROS_WARN("Serial port occupied. ");
			continue;
		}

		if(r_buffer[0] != R_HEADER){
			ROS_WARN("Not a ROS receive message!");
			continue;
		}

		if(r_buffer[1] == CHASSIS_VEL_HEADER){
			for(i=0;i<2;i++){
				vx.cvalue[i] = r_buffer[3-i];
				va.cvalue[i] = r_buffer[5-i];
			}
			//Get ROS time
			current_time = ros::Time::now();
			//odom timestamp
			odom.header.stamp = current_time;
			
			odom.header.frame_id = "odom";
			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = (double)vx.sivalue/1000;
			odom.twist.twist.linear.y = 0;
			odom.twist.twist.angular.z = (double)va.sivalue/1000;
			//
			odom_pub.publish(odom);
			ROS_INFO("publish odometry");
			last_time = current_time;
		}else if(r_buffer[1] == CHASSIS_POS_HEADER){
			for(i=0;i<4;i++){
				posx.cvalue[i] = r_buffer[5-i];
				posy.cvalue[i] = r_buffer[9-i]; 
			}

			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";
			current_time = ros::Time::now();
			odom_trans.header.stamp = current_time;

			odom_trans.transform.translation.x = (double)posx.ivalue/1000;//x
			odom_trans.transform.translation.y = (double)posy.ivalue/1000;//y
			odom_trans.transform.translation.z = 0;
			odom_broadcaster.sendTransform(odom_trans);

			odom.pose.pose.position.x = (double)posx.ivalue/1000;
			odom.pose.pose.position.y = (double)posy.ivalue/1000;
			odom.pose.pose.position.z = 0;
		}else if(r_buffer[1] == CHASSIS_ORI_HEADER){
			for(i=0;i<4;i++){
				yaw.cvalue[i] = r_buffer[5-i];
			}
			//Convert the Euler angles to a quaternion
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "base_link";
			current_time = ros::Time::now();
			odom_trans.header.stamp = current_time;

			odom_quat = tf::createQuaternionMsgFromYaw((double)yaw.ivalue/1000);
			odom_trans.transform.rotation = odom_quat;//q
			odom_broadcaster.sendTransform(odom_trans);

			odom.pose.pose.orientation = odom_quat;
		}else if(r_buffer[1] == CHASSIS_TAU_HEADER){
			for(i=0;i<2;i++){
				tau_l.cvalue[i] = r_buffer[3-i];
				tau_r.cvalue[i] = r_buffer[5-i];
			}
			wheel_torque.tau_l.data = (float)tau_l.sivalue/1000;
			wheel_torque.tau_r.data = (float)tau_r.sivalue/1000;

			wheel_torque_pub.publish(wheel_torque);
		
			/*ROS_INFO("chassis buffer");
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
			last_time = current_time;*/
		}else if(r_buffer[1] == BUCKET_MODE_HEADER){
			bucket_msg.mode.data = static_cast<uint8_t>(r_buffer[2]);
			for (i=0;i<2;i++){
				ext1.cvalue[i] = r_buffer[4-i];
				ext2.cvalue[i] = r_buffer[6-i];
				load.cvalue[i] = r_buffer[8-i];
			}
			bucket_msg.L1_extension.data = (float)ext1.sivalue/1000;
			bucket_msg.L2_extension.data = (float)ext2.sivalue/1000;
			bucket_msg.load.data = (float)load.sivalue/1000;

			bucket_pub.publish(bucket_msg);
		}else if(r_buffer[1] == BUCKET_DYN_HEADER){
			for (i=0;i<2;i++){
				f1l.cvalue[i] = r_buffer[3-i];
				f2r.cvalue[i] = r_buffer[5-i];
				f1r.cvalue[i] = r_buffer[7-i];
				f2r.cvalue[i] = r_buffer[9-1];
			}
			// calculate average or change ROS message? 
			bucket_msg.L1_force.data = (double)(f1l.sivalue + f1r.sivalue)/2000;
			bucket_msg.L2_force.data = (double)(f2l.sivalue + f2r.sivalue)/2000;

			bucket_pub.publish(bucket_msg);
		}else if(r_buffer[1] == UTIL_MODE_HEADER){
			util_msgs.horn.data = (bool)r_buffer[2];
			util_msgs.headlight.data = (bool)r_buffer[3];
			util_msgs.emergency_stop.data = (bool)r_buffer[4];
			util_msgs.isCharging.data = (bool)r_buffer[5];
			util_msgs.remote_mode.data = (bool)r_buffer[6];
			util_msgs.battery_percentage.data = (uint8_t)r_buffer[7];
			util_pub.publish(util_msgs);
		}else if(r_buffer[1] == ERROR_HEADER){

		}else {
			ROS_WARN("Invalid message! ");
		}

		memset(r_buffer,0,BUFFERSIZE);
        loop_rate.sleep();

    }
}


int comm_init()
{
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
    
	ros::NodeHandle nh;
	
	//subscribe ROS topics
	bucket_sub = nh.subscribe("/bucket_cmd",1000,bucket_msg_callback);
	cmd_vel_sub = nh.subscribe("/cmd_vel",1000,cmd_vel_callback); 
	util_sub = nh.subscribe("/util_cmd",1000,util_msg_callback);
	//publish ROS topics
	odom_pub = nh.advertise<nav_msgs::Odometry>("odom",1000);
	bucket_pub = nh.advertise<accr_msgs::Bucket>("bucket_msg", 1000);
	wheel_torque_pub = nh.advertise<accr_msgs::Chassis>("wheel_torque",1000);
	util_pub = nh.advertise<accr_msgs::Utility>("util_msgs",1000);
	// error_pub = nh.advertise<accr_msgs::Error>("error_msgs", 1000);

	current_time = ros::Time::now();
	last_time = ros::Time::now();
}

void mc_init()
{
	memset(s_init_buffer, 0, BUFFERSIZE);
	s_init_buffer[0] = S_HEADER;
	s_init_buffer[1] = INIT_HEADER;
	s_init_buffer[2] = init_op.ts>>8;
	s_init_buffer[3] = init_op.ts;
	s_init_buffer[4] = init_op.freq_div;
	s_init_buffer[5] = init_op.chassis_op;
	s_init_buffer[6] = init_op.bucket_op;
	s_init_buffer[7] = init_op.other_op;
	s_init_buffer[9] = init_op.reset_op;
	ser.write(s_init_buffer, BUFFERSIZE);
}

// Send buffer ---
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel)
{
	//ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	//ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;
	int16_t vel_x = static_cast<int16_t>(cmd_vel.linear.x*1000);
	int16_t ang_v = static_cast<int16_t>(cmd_vel.angular.z*1000);
	//ROS_INFO("linear vel: x-[%d]",vel_x);
	//ROS_INFO("angular vel: [%d]",ang_v);
	memset(s_chassis_buffer, 0, BUFFERSIZE);
	s_chassis_buffer[0] = S_HEADER;
	s_chassis_buffer[1] = CHASSIS_VEL_HEADER;
	//linear ref speed [mm/s]
	s_chassis_buffer[2] = vel_x>>8;
	s_chassis_buffer[3] = vel_x;
	//anguar ref speed [mrad/s]
	s_chassis_buffer[4] = ang_v>>8;
	s_chassis_buffer[5] = ang_v;
	
	ser.write(s_chassis_buffer,BUFFERSIZE);
	// Byte 6,7,8,9 are left unused. 
	
	// for(int i=0;i<15;i++){
	// 	ROS_INFO("0x%02x",s_chassis_buffer[i]);
	// }
}

void bucket_msg_callback(const accr_msgs::Bucket& bucket_cmd)
{
	//int16_t   bucket_joint = bucket_msg.bucket_joint_angle.data;
	//ROS_INFO("I heard bucket: x-[%d],",bucket_msg.bucket_joint_angle.data);
	int16_t ext1 = static_cast<int16_t>(bucket_cmd.L1_extension.data*1000);
	int16_t ext2 = static_cast<int16_t>(bucket_cmd.L2_extension.data*1000);

	memset(s_bucket_buffer, 0, BUFFERSIZE);
	s_bucket_buffer[0] = S_HEADER;
	s_bucket_buffer[1] = BUCKET_MODE_HEADER;
	s_bucket_buffer[2] = static_cast<unsigned char>(bucket_cmd.mode.data);    //Set bucket control mode
	//bucket arm length cmd
	s_bucket_buffer[3] = ext1>>8;
	s_bucket_buffer[4] = ext1;
	s_bucket_buffer[5] = ext2>>8;
	s_bucket_buffer[6] = ext2;	
	// Byte 7,8,9 are left unused. 

	ser.write(s_bucket_buffer, BUFFERSIZE);
	// for(int i = 0; i<3; i++)
	// {
	// 	ser.write(s_bucket_buffer, BUFFERSIZE);
	// 	ros::Duration(0.05).sleep();
	// }
}

void util_msg_callback(const accr_msgs::Utility& util_cmd)
{
	unsigned char horn = static_cast<unsigned char>(util_cmd.horn.data);
	unsigned char headlight = static_cast<unsigned char>(util_cmd.headlight.data);

	memset(s_util_buffer, 0, BUFFERSIZE);
	s_util_buffer[0] = S_HEADER;
	s_util_buffer[1] = UTIL_MODE_HEADER;

	s_util_buffer[2] = horn;  //horn switch 
	s_util_buffer[3] = headlight; //headlight switch

	// Byte 4-9 are left unused. 

	ser.write(s_util_buffer, BUFFERSIZE);
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





