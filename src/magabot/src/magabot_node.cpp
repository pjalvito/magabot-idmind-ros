
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel

#include <cereal_port/CerealPort.h>


// MAGABOT Dimensions


			
#ifndef NORMALIZE
    #define NORMALIZE(z) atan2(sin(z), cos(z))	// Normalize angle to domain -pi, pi 
#endif

double MAGABOT_WIDTH = 0.345;  	//m. Between wheels
double WHEEL_RADIUS = 0.043937;	//m
double TICKS_PER_TURN_L = 3900;//2450;
double TICKS_PER_TURN_R = 2230;
//MAGABOT data for odometry


double TWOPI  = 6.2831853070;
double PI = 3.1415926535;

const float K_MOTOR_RATIO = TICKS_PER_TURN_L * 0.00332 / ( PI * WHEEL_RADIUS);
ros::Publisher *odom_pub;
tf::TransformBroadcaster *odom_broadcaster;

cereal::CerealPort serial_port, serial_port_inertial;

ros::Time current_time, last_time;
std::string base_frame_id;
std::string odom_frame_id;
double last_time_pub = 0.0;

bool signof (int n) { return n >= 0; }
bool confirm_communication = true;
int ID_Robot = 0;

double odometry_x = 0.0;
double odometry_y = 0.0;
double odometry_yaw = 0.0; 
double odometry_pitch = 0.0;
double odometry_roll = 0.0;
int right_encoder_prev = 0;
int left_encoder_prev = 0;
int n_inter = 0;
double dt = 0;
double vel_x = 0;
double vel_y = 0;
double vel_yaw = 0;
bool to_write = false;
double yaw_default = 0;
int left_encoder_count = 0;
int right_encoder_count = 0;
int left_encoder_acc = 0;
int right_encoder_acc = 0;
double x,y,th;
int n_iter = 0;
float correction = 0;
int lvel = 0;
int rvel = 0;



void drive(double linear_speed, double angular_speed) 
{
	float ang = angular_speed * MAGABOT_WIDTH/2;
	int left_write = 0;
	int right_write = 0;
	if(linear_speed == 0 && angular_speed == 0)
	{
		left_write = 0;
		right_write = 0;
	}
	else
	{
		float l_ratio = K_MOTOR_RATIO* 0.5543808461*pow(fabs(linear_speed - ang),-0.1423547731);
		float r_ratio = K_MOTOR_RATIO* 0.5543808461*pow(fabs(linear_speed + ang),-0.1423547731);
	
		right_write = (int)((linear_speed + (angular_speed * MAGABOT_WIDTH / 2)) * r_ratio);
		left_write = (int)((linear_speed - (angular_speed * MAGABOT_WIDTH / 2)) * l_ratio);
	}
	//ROS_FATAL("[ref_values]: Vl = %d ; Vr = %d",left_write, right_write);

	if(left_write < 6 && left_write > 0)
	{
		left_write = 7;
	}

	if(left_write > -6 && left_write < 0)
	{
		left_write = -7;
	}
	if(right_write < 6 && right_write > 0)
	{
		right_write = 7;
	}

	if(right_write > -6 && right_write < 0)
	{
		right_write = -7;
	}

	
	lvel = left_write;
	rvel = right_write;
	
}
void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    	//ROS_FATAL("[cmd_vel]: Vlinear = %f, Vangular = %f",cmd_vel->linear.x, cmd_vel->angular.z);
	drive(cmd_vel->linear.x, cmd_vel->angular.z);
}

void getSonars()
{
    char sonars_command[1];
    sonars_command[0] = char(0x83);
    serial_port.write(sonars_command,1);
    char sonars_response[10];
    serial_port.read(sonars_response, 10, 1000);
    int sonars0 = ((int)(unsigned char)sonars_response[0] * 255 + (int)(unsigned char)sonars_response[1]);
    int sonars1 = ((int)(unsigned char)sonars_response[2] * 255 + (int)(unsigned char)sonars_response[3]);
    int sonars2 = ((int)(unsigned char)sonars_response[4] * 255 + (int)(unsigned char)sonars_response[5]);
    int sonars3 = ((int)(unsigned char)sonars_response[6] * 255 + (int)(unsigned char)sonars_response[7]);
    int sonars4 = ((int)(unsigned char)sonars_response[8] * 255 + (int)(unsigned char)sonars_response[9]);

	//ROS_FATAL("SONARS:%d %d %d %d %d", sonars0, sonars1, sonars2, sonars3, sonars4);
}

void getIR()
{
    char ir_command[1];
    ir_command[0] = char(0x49);
    serial_port.write(ir_command,1);
    char ir_response[6];
    serial_port.read(ir_response, 6, 1000);
    int ir0 = ((int)(unsigned char)ir_response[0]*255 + (int)(unsigned char)ir_response[1]);
    int ir1 = ((int)(unsigned char)ir_response[2]*255 + (int)(unsigned char)ir_response[3]);
    int ir2 = ((int)(unsigned char)ir_response[4]*255 + (int)(unsigned char)ir_response[5]);
	//ROS_FATAL("IRS:%d %d %d", ir0, ir1, ir2);

}

void getBattery()
{
    char bat_command[1];
    bat_command[0] = char(0x4B);
    serial_port.write(bat_command,1);
    char bat_response[2];
    serial_port.read(bat_response, 2, 1000);
    int bat_level = ((int)bat_response[0]*255 + (int)bat_response[1]);
    //ROS_FATAL("BATTERY LEVEL %d", bat_level);
}

bool getBumpers()
{
    bool _bump = false;
    char bumper_command[1];
    bumper_command[0] = char(0x66);
    serial_port.write(bumper_command,1);
    char bumper_response[2];
    serial_port.read(bumper_response, 2, 1000);
    if(bumper_response[0] == 1 || bumper_response[1] == 1)
    {
	_bump = true;
	//ROS_FATAL("BUMP");
    }
    
    return _bump;
    
}

void publish_odometry(int l_ticks, int r_ticks)
{
	double last_x = odometry_x;
	double last_y = odometry_y;
	double last_yaw = odometry_yaw;  
	last_time = current_time;
	current_time = ros::Time::now();
	dt = (current_time - last_time).toSec();
	bool publish_info = true;
	//ROS_FATAL("Num ticks acumulados: %i , %i", l_ticks, r_ticks);
	
	double dl = -(double)(l_ticks * 3.14 * 2 * WHEEL_RADIUS /TICKS_PER_TURN_L);
	double dr = (double)(r_ticks * 3.14 * 2 * WHEEL_RADIUS / TICKS_PER_TURN_L);	
	double dx = (dr + dl) /2.0;
	//double angle = (double)((dr - dl)/MAGABOT_WIDTH);
	//odometry_yaw = -(ypr[0]);	 			//rad
	//double aux = odometry_yaw + angle;
	//odometry_yaw = atan2(sin(aux), cos(aux));						//rad
	odometry_x += dx * cos((double) odometry_yaw);   //m
	odometry_y += dx * sin((double) odometry_yaw);	//m

	//double dt = (current_time - last_time).toSec();
	double vel_x = dx/dt;
	double vel_y = 0;
	double vel_yaw = (odometry_yaw - last_yaw)/dt;
	
	if(vel_x != 0)
		ROS_FATAL("[odometry_vel]: vx = %f vrot = %f",vel_x, vel_yaw);

// ******************************************************************************************		//first, we'll publish the transforms over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
			
	odom_trans.transform.translation.x = odometry_x;
	odom_trans.transform.translation.y = odometry_y;


	odom_trans.transform.translation.z = 0.0;


	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odometry_yaw);
	odom_broadcaster->sendTransform(odom_trans);
		
	// ******************************************************************************************
	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";
		
	//set the position
	odom.pose.pose.position.x = odometry_x;
	odom.pose.pose.position.y = odometry_y;
	odom.pose.pose.position.z = 0.0;

 	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odometry_yaw);
//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vel_x;
	odom.twist.twist.linear.y = vel_y;
	odom.twist.twist.angular.z = vel_yaw;
	/*odom.pose.covariance[0] = 0.001;
	odom.pose.covariance[7] = 0.001;
	odom.pose.covariance[14] = 0.001;
	odom.pose.covariance[21] = 1000;
	odom.pose.covariance[28] = 1000;
	odom.pose.covariance[35] = 1000;

	odom.twist.covariance = odom.pose.covariance;	*/
	//publish the odometry
	odom_pub->publish(odom);
}

//Receive encoder ticks and send 'odom' and 'tf'
int getRPY()
{
	char command[1];
	command[0] = (char)0x40;
	
	try
	{ 
		serial_port_inertial.write(command, 1); 
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("ERROR writing INERTIAL_CMD in serial port");
		return -1;
	}

	int Yaw_int, Pitch_int, Roll_int, Checksum_read,Checksum_Total ;
	int size = 10;
	char buffer[size];
	try
	{ 
		serial_port_inertial.readBytes(buffer, size, 1000); 
	}
	catch(cereal::Exception& e)
	{ 
		ROS_ERROR("ERROR reading INERTIAL from serial port");
		
		return(-1); 
	}
	Yaw_int = (int)((int)buffer[1] << 8) + (int)(buffer[2]);
        if (Yaw_int > 32767) 
		Yaw_int = -(65535 - Yaw_int);

	Pitch_int = (int)((int)buffer[3] << 8) + (int)(buffer[4]);
        if (Pitch_int > 32767) 
		Pitch_int = -(65535 - Pitch_int);

	Roll_int = (int)((int)buffer[5] << 8) + (int)(buffer[6]);
        if (Roll_int > 32767) 
		Roll_int = -(65535 - Roll_int);

	odometry_pitch = (double)Pitch_int / 100.0;
	odometry_roll = (double)Roll_int / 100.0;
        odometry_yaw = -(double)Yaw_int / 100.0;	//yaw provided in angles
	odometry_yaw *= M_PI / 180.0;			//converted to rads
       
	ROS_FATAL("YAW:%f", odometry_yaw);   
	return 0;
}
void timerCallback(const ros::TimerEvent&)
{
	getRPY();
	
	getSonars();
	

	//GET ODOMETRY	
	char tick_command[1];
	tick_command[0] = char(0x74);
	serial_port.write(tick_command,1);
	char tick_response[4];
	serial_port.read(tick_response, 4, 1000);
	int _l_ticks = ((int)tick_response[0]*256 + (int)tick_response[1]);
	int _r_ticks = ((int)tick_response[2]*256 + (int)tick_response[3]);
	//ROS_FATAL("ODOM:%d %d", _l_ticks, _r_ticks);	
	publish_odometry(_l_ticks, _r_ticks);
	
	getIR();	
	getBattery();	
	getBumpers();
	
	
	//WRITE VELOCITIES
	
	char vel_command[5];
	vel_command[0] = (char)(0x86);
	vel_command[1] = (char)((int)(abs(lvel)));
	if(lvel > 0)
		vel_command[2] = (char)(0);
	else
		vel_command[2] = (char)(1);

	vel_command[3] = (char)((int)(abs(rvel)));
	
	if(rvel > 0)
		vel_command[4] = (char)(0);
	else
		vel_command[4] = (char)(1);	
		
	serial_port.write(vel_command,5);
//ROS_FATAL("%d %d", (int)(abs(lvel)), (int)(abs(rvel)));
	//char vel_response[1];
	//serial_port.read(vel_response, 1, 1000);
}

//receive cmds_vel from nav_stack


int main(int argc, char** argv){ 
  
	ros::init(argc, argv, "magabotnode");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	ros::NodeHandle pni("~");
	std::string port, portinertial;
	
	if (argc<2)
	{
		port="/dev/ttyACM0";
		portinertial="/dev/ttyACM1";
		ROS_WARN("No Serial Port defined, defaulting to \"%s\"",port.c_str());
		ROS_WARN("Usage: \"rosrun [pkg] robot_node /serial_port\"");
	}
	else
	{
		port="/dev/ttyACM0";
		portinertial="/dev/ttyACM1";
		ROS_INFO("Serial port: %s",port.c_str());
	}	
	


	pn.param<std::string>("base_frame_id", base_frame_id, "base_link");
	pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");
	
	// ROS publishers and subscribers

	ros::Publisher odom_pub_ptr = n.advertise<nav_msgs::Odometry>("/odom", 500);
    	tf::TransformBroadcaster odom_broadcaster_ptr;
 	ros::Subscriber cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, cmdVelReceived);
        ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
	odom_pub = &odom_pub_ptr;
	odom_broadcaster = &odom_broadcaster_ptr;	
 	
	
	// baud_rate and serial port:	
	int baudrate;
	pn.param<std::string>("port", port, port.c_str()); 
	pn.param("baudrate", baudrate, 9600); 

	// Open the serial port to the robot
	try
	{ 
		serial_port.open((char*)port.c_str(), baudrate); 
	}
	catch(cereal::Exception& e)
	{
		ROS_FATAL("Robot -- Failed to open odometry serial port!");
		ROS_BREAK();
	}
ros::Duration(2.5).sleep(); 
	 int baudrate2;
	pn.param<std::string>("port_inertial", portinertial, portinertial.c_str()); 
	pn.param("baudrate2", baudrate2, 115200); 
	
	try
	{ 
		serial_port_inertial.open((char*)portinertial.c_str(), 	baudrate2); 
		char command[2];
		command[0] = (char)0x50;
		command[1] = (char)0x02;
		serial_port_inertial.write(command, 2);
		char buffer[4];
		try
		{
			serial_port_inertial.readBytes(buffer, 4, 1000);
		}
		catch(cereal::Exception& e)
		{
			ROS_ERROR("ERROR writing INERTIAL_CMD in serial port");
		}
	}
	catch(cereal::Exception& e)
	{
		ROS_FATAL("Robot -- Failed to open inertial serial port!");
		//ROS_BREAK();
	}

    
	
	//wait (2.5 seconds) until serial port gets ready
	ros::Duration(2.5).sleep(); 	
    
	ros::spin(); //trigger callbacks and prevents exiting
  	return(0);
}


