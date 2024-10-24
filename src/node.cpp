#include "geometry_msgs/TransformStamped.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_server.h"
#include "ros/spinner.h"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include "tf/LinearMath/Transform.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "ximu3_ros/Connection_ros.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <functional>

int get_divisor(std::string param_name,ros::NodeHandle nh)
{
	int divisor_rate;
	if (nh.getParam(param_name + "_divisor", divisor_rate))
	{
		if (divisor_rate > 0)
		{
			ROS_INFO_STREAM("Setting" << param_name << " divisor rate to:"<< divisor_rate);
			ROS_INFO_STREAM("Will publish "<< param_name <<" at rate of: " << 400.0/divisor_rate);
		}
		else
		{
			ROS_ERROR_STREAM(param_name << " divisor set to zero disables the XXX. No YYYY will be published. Are you sure this is what you want?");
		}
	}
	else
	{
		divisor_rate = 8;
		ROS_WARN_STREAM("Using default "<< param_name << " divisor rate: " << divisor_rate );
	}

	return divisor_rate;
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "imu_reader_node", ros::init_options::AnonymousName);
	//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	//	ros::console::notifyLoggerLevelsChanged();
	//}
	ros::NodeHandle nh("~");
	std::string parent_frame_id;
	std::vector<double> origin;
	bool publish_status;
	ros::Publisher bat_pub, bat_v_pub, temp_pub, imu_pub;
	if(nh.getParam("parent_frame_id", parent_frame_id))
	{
		ROS_INFO("Using parent frame_id: %s", parent_frame_id.c_str());
	}
	else
	{
		//parent_frame_id = "world";
		parent_frame_id = "map";
	}
	std::string calibration_frame_id;
	if(nh.getParam("calibration_frame_id", calibration_frame_id))
	{
		ROS_INFO("Using as calibration frame_id: %s", calibration_frame_id.c_str());
	}
	else
	{
		//calibration_frame_id = "world";
		calibration_frame_id = "map";
	}

	std::string own_tf_name;
	if(nh.getParam("name", own_tf_name))
	{
		ROS_INFO("Publishing child_frame_id: %s", own_tf_name.c_str());
	}
	else
	{
		own_tf_name = "ximu";
		ROS_ERROR("Failed to get param 'name'. Setting default child_frame_id to '%s'.", own_tf_name.c_str());
	}
	std::string ip_address;
	if(nh.getParam("ip_address", ip_address))
	{
		ROS_INFO("Accessing imu data from %s", ip_address.c_str());
	}
	else
	{
		ip_address = "192.168.1.1";
		ROS_FATAL("Need an ip address for the imu.");
		return -1;
	}
	int receive_port, send_port;
	if(nh.getParam("receive_port", receive_port))
	{
		ROS_INFO("Reading from port: %d", receive_port);
	}
	else
	{
		receive_port = 9000;
		ROS_WARN("Using default port: %d. ", receive_port);
		ROS_WARN("Make sure port is unique and exposed in docker (both dockerfile and command line call!)");
	}
	if (nh.getParam("send_port", send_port))
	{
		ROS_INFO("Sending to port %d", send_port);
	}
	else
	{
		send_port = 8001;
		ROS_WARN("Using default send_port: %d", send_port );
		//ROS_WARN("Make sure port is unique and exposed in docker (both dockerfile and command line call!)");
	}

	int ahrs_divisor_rate = get_divisor("ahrs", nh);
	int sensors_divisor_rate = get_divisor("sensor", nh);

	nh.getParam("origin", origin);
	if (origin.size() == 0)
	{
		origin = {0,0,0};
		ROS_INFO("Origin not set. Using 0,0,0.");
	}

	if (nh.getParam("publish_status", publish_status))
	{
		ROS_INFO("Publishing status messages");
		temp_pub = nh.advertise<std_msgs::Float32>("temperature",1);
		bat_pub = nh.advertise<std_msgs::Float32>("battery/percentage",1);
		bat_v_pub = nh.advertise<std_msgs::Float32>("battery/voltage",1);
	}
	imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);

	bool do_calib = false;
	nh.getParam("do_calibration", do_calib);

	bool wait_to_start = false;
	nh.getParam("wait_to_start", wait_to_start);

	bool use_imu_time_stamps = true;
	nh.getParam("use_imu_time_stamps", use_imu_time_stamps);
	if (use_imu_time_stamps)
	{
		ROS_WARN("Using XIMU internal clock to publish ros timestamps. Note: you probably also want to add some offset to guarantee that no timestamp is in the future. If this fails, you know why. I am not sure what this offset should be, maybe 10 ms or maybe 10ms - 1/2*the period of the thing being read?");
	}
	else
	{
		ROS_WARN("Using PC clock for stamping messages. Note: This was not documented in the first versions of the XIMU3 manual, but the highest transmission rate is 100Hz.  In this mode you cannot run anything at higher rates than 100Hz, because you get them at the same time in a burst. ");
	}


	ROS_DEBUG_STREAM("before you get mad at me that the imu messages are not being published at 400 hz when you may set it to 400 hz, currently the way that it is being done is that, only when each time you get new linear speeds and gyro messages are received that you publish a new imu_msg. You could maybe try to publish always the most update one and do something fancy like having threads and updating a common latest message and then publishing that every time either one of the fields changed. This is left as an exercise to the reader.");
	if (wait_to_start)
	{
		ros::Rate wait_rate(5);
		bool start_now = false;
		boost::function<bool(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)> wait_callback = [&start_now](auto req, auto res) -> bool
		{
			start_now = true;
			return true;
		};
		//create a service, wait for that service call to change state so we can start
		//
		ros::ServiceServer ss = nh.advertiseService("start_now", wait_callback);

		ROS_WARN("not_tested");
		while (!start_now)
		{
			ROS_INFO_STREAM_THROTTLE(1,"Waiting to start");
			wait_rate.sleep();
			ros::spinOnce();
		}
	}


	bool dummy_publisher=false;
	nh.getParam("dummy_publisher", dummy_publisher);
	if (dummy_publisher)
	{
		ROS_WARN_STREAM("Running dummy_publisher!!! Not attempting to connect to imu: "<<own_tf_name);
		ros::Rate rr(400.0/ahrs_divisor_rate);
		auto diags_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",5);	
		diagnostic_msgs::DiagnosticStatus delay_msg, temperature_msg, percentage_msg;
		delay_msg.name = own_tf_name+"/delay";
		delay_msg.level = diagnostic_msgs::DiagnosticStatus::OK;

		tf2_ros::TransformBroadcaster br;
		//this is missing the tfs...
		while(ros::ok())
		{
			sensor_msgs::Imu imu_msg;
			geometry_msgs::TransformStamped t;
			t.child_frame_id = own_tf_name;
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.header.frame_id = parent_frame_id;
			imu_msg.orientation.w =1; // so that it isnt an unnormalized quaternion
			imu_pub.publish(imu_msg);
			t.header = imu_msg.header;
			t.transform.rotation = imu_msg.orientation;
			diagnostic_msgs::DiagnosticArray a_diags_msg;
			a_diags_msg.header = imu_msg.header;
			a_diags_msg.status.push_back(delay_msg);
			diags_pub.publish(a_diags_msg);
			br.sendTransform(t);
			rr.sleep();
			ros::spinOnce();
		}
	}
	else
	{
		Connection c(calibration_frame_id, parent_frame_id, own_tf_name, ahrs_divisor_rate, sensors_divisor_rate, origin, temp_pub, bat_pub, bat_v_pub, imu_pub, publish_status, nh, do_calib, use_imu_time_stamps);
		//c.run(ximu3::UdpConnectionInfo("192.168.1.1", 9000, 8001));
		ros::ServiceServer calib_srv = nh.advertiseService("calibrate", &Connection::calibrationSrv, &c);
		c.run(ximu3::UdpConnectionInfo(ip_address, receive_port, send_port));
	}
	return 0;
}

