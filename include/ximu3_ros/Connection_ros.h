#pragma once

#include "ros/duration.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_server.h"
#include "ros/time.h"
#include "std_srvs/Empty.h"
#include "tf2/convert.h"
#include "ximu3_ros/Connection.hpp"
#include "ximu3_ros/Ximu3.hpp"
#include "ximu3_ros/Helpers_api.hpp"
#include <chrono>
#include <inttypes.h> // PRIu64
#include <iostream>
#include <ostream>
#include <sstream>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <thread>
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/Imu.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"

#include <limits>
constexpr int32_t max_int32 = std::numeric_limits<int32_t>::max();


#define TIMESTAMP_FORMAT "%8" PRIu64 " us"
#define UINT32_FORMAT " %8" PRIu32
#define UINT64_FORMAT " %8" PRIu64
#define FLOAT_FORMAT " %8.3f"
#define STRING_FORMAT " \"%s\""

class ConnectionParams
{
	public:
		std::string child_frame_id;
		std::string imu_name;
		std::string parent_frame_id;
		unsigned int ahrs_divisor_rate = 8;
		std::vector<double> origin;
		bool publish_status;
		bool do_calibration = true;
};


class Connection
{
	public:
		//ros::Publisher poser_pub;
		tf2_ros::TransformBroadcaster br;
		float time_delay_monitor, battery_percentage_monitor;
		//protected:
		ros::WallTime last_time;
		ros::WallTime initial_time;
		uint64_t last_time_stamp = 0;
		uint64_t this_time_stamp = 0;
		uint64_t initial_time_stamp = 0;
		ximu3::Connection* connection;
		std::string child_frame_id;
		std::string imu_name;
		std::string parent_frame_id;
		unsigned int my_divisor_rate = 8;
		unsigned int my_other_divisor_rate = 8;
		std::vector<double> origin;
		std::optional<tf2::Quaternion> q_cal;
		diagnostic_msgs::DiagnosticStatus d_msg;
		diagnostic_msgs::DiagnosticArray da_msg;
		//tf2::Quaternion q_cal{0,0,0,1};
		bool publish_status;
		bool do_calibration = true;
		bool use_imu_time_stamps = true;
		ros::Publisher bat_pub, bat_v_pub, temp_pub, imu_pub, diags_pub;
		ros::ServiceServer set_heading_srv;
		void enable_magnetometer()
		{
			ROS_INFO_STREAM("Enabling magnetometer!");
			const std::vector<std::string> heading_commands1 {"{\"ahrsIgnoreMagnetometer\":false}", "{\"ahrsAccelerationRejectionEnabled\":false}"};
			connection->sendCommands(heading_commands1,2,500);
			apply_save();
		}
		void disable_magnetometer()
		{
			ROS_INFO_STREAM("Disabling magnetometer!");
			const std::vector<std::string> heading_commands1 {"{\"ahrsIgnoreMagnetometer\":true}" };
			connection->sendCommands(heading_commands1,2,500);
			apply_save();

		}
		void apply_save()
		{
			const std::vector<std::string> apply_commands { "{\"apply\":null}" };
			connection->sendCommands(apply_commands, 2, 500);
			ROS_DEBUG_STREAM("Applying command on IMU");
			//sometimes it doesnt work without this? I dont want to test it. Maybe it can be removed.
			const std::vector<std::string> save_command { "{\"save\":null}" };
			connection->sendCommands(save_command, 2, 500);
			ROS_DEBUG_STREAM("Saving settings on IMU EEPROM");

		}
		bool set_heading_DOESNT_WORK(std_srvs::Empty::Request & req,
				std_srvs::Empty::Response & res)
		{
			ROS_INFO_STREAM("called set heading!");
			const std::vector<std::string> heading_commands1 {"{\"ahrsIgnoreMagnetometer\":true}" };
			connection->sendCommands(heading_commands1,1,1);
			ROS_DEBUG_STREAM("sending heading heading_commands1 done ok!");
			const std::vector<std::string> heading_commands2 {"{\"heading\":0}\r\n" };
			connection->sendCommands(heading_commands2,2,500);
			ROS_DEBUG_STREAM("sending heading commands2 done ok!");
			const std::vector<std::string> apply_commands { "{\"apply\":null}" };
			connection->sendCommands(apply_commands, 2, 500);
			ROS_DEBUG_STREAM("Applying command on IMU");
			//sometimes it doesnt work without this? I dont want to test it. Maybe it can be removed.
			const std::vector<std::string> save_command { "{\"save\":null}" };
			connection->sendCommands(save_command, 2, 500);
			ROS_DEBUG_STREAM("Saving settings on IMU EEPROM");
			// Send command to strobe LED
			const std::vector<std::string> commands { "{\"strobe\":null}" };
			connection->sendCommands(commands, 2, 500);
			return true;
		}
		Connection(): child_frame_id("ximu3"), parent_frame_id("map"), my_divisor_rate(8)

	{}
		Connection(std::string parent_frame_id_, std::string child_frame_id_, unsigned int div, unsigned int div_sensors, std::vector<double> origin_, ros::Publisher temp_pub_, ros::Publisher bat_pub_, ros::Publisher bat_v_pub_, ros::Publisher imu_pub_, bool publish_status_ , ros::NodeHandle nh, bool do_calibration_, bool use_imu_time_stamp_):use_imu_time_stamps(use_imu_time_stamp_), do_calibration(do_calibration_), child_frame_id(child_frame_id_), parent_frame_id(parent_frame_id_), my_divisor_rate(div), my_other_divisor_rate(div_sensors), origin(origin_), bat_pub(bat_pub_), bat_v_pub(bat_v_pub_), temp_pub(temp_pub_), publish_status(publish_status_), imu_pub(imu_pub_)
	{
		imu_name = child_frame_id; // maybe I should read something fancy here to better identify them
		ROS_INFO("Parent frame_id set to %s", parent_frame_id.c_str());
		ROS_INFO("Child frame_id set to %s", child_frame_id.c_str());
		diags_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);	
		d_msg.name = imu_name;
		da_msg.header.frame_id = parent_frame_id;
		//q_cal = {0.5,0.5,0.5,0.5};
		//set_heading_srv = nh.advertiseService("set_heading", &Connection::set_heading, this);
		if (! do_calibration)
		{
			q_cal = {0,0,0,1};
			ROS_INFO_STREAM("No calibration.");
		}
	}
		void set_rate()
		{

			const std::vector<std::string> rate_commands { 
				"{\"inertialMessageRateDivisor\":"+std::to_string(my_other_divisor_rate)+"}", 
				"{\"highGAccelerometerMessageRateDivisor\":0}",
				"{\"ahrsMessageRateDivisor\":"+std::to_string(my_divisor_rate)+"}" 
			};
			for (auto a:rate_commands)
				ROS_DEBUG_STREAM("Setting command:" << a);
			connection->sendCommands(rate_commands, 2, 500);
			const std::vector<std::string> apply_commands { "{\"apply\":null}" };
			connection->sendCommands(apply_commands, 2, 500);
			ROS_DEBUG_STREAM("Applying command on IMU");
			//sometimes it doesnt work without this? I dont want to test it. Maybe it can be removed.
			const std::vector<std::string> save_command { "{\"save\":null}" };
			connection->sendCommands(save_command, 2, 500);
			ROS_DEBUG_STREAM("Saving settings on IMU EEPROM");


		}
		void run(const ximu3::ConnectionInfo& connectionInfo)
		{
			ros::Rate r(1);
			initial_time = ros::WallTime::now();
			last_time = initial_time;
			//ros::NodeHandle n;
			//poser_pub = n.advertise<geometry_msgs::PoseStamped>("poser", 1000);

			// Create connection
			connection = new ximu3::Connection(connectionInfo);
			if (do_calibration)
			{
				disable_magnetometer();
			}
			else
			{
				enable_magnetometer();
			}
			connection->addDecodeErrorCallback(decodeErrorCallback);
			connection->addStatisticsCallback(statisticsCallback);
			{
				connection->addQuaternionCallback(quaternionCallback);
				connection->addNotificationCallback(notificationCallback);
				connection->addInertialCallback(inertialCallback);
				//status messages
				if (publish_status)
				{
					connection->addBatteryCallback(batteryCallback);
					connection->addTemperatureCallback(temperatureCallback);
				}
			}

			// Open connection
			ROS_INFO_STREAM("Connecting to " << connectionInfo.toString());
			if (connection->open() != ximu3::XIMU3_ResultOk)
			{
				ROS_ERROR_STREAM( "Unable to open connection");
				return;
			}
			ROS_INFO_STREAM("Connection successful");

			set_rate();

			//attempt at reading status of ahrs magnetometer. no worko.
			//auto response_question_mark =(connection->sendCommands({"{\"ahrsIgnoreMagnetormeter\":null}"}, 1,1));
			//for (auto res_res:response_question_mark)
			//	ROS_INFO_STREAM(res_res);
			// Send command to strobe LED
			const std::vector<std::string> commands { "{\"strobe\":null}" };
			connection->sendCommands(commands, 2, 500);

			// Close connection
			//helpers::wait(-1);
			while(ros::ok())
			{
				ros::WallTime this_time = ros::WallTime::now();
				double execution_time = (this_time - last_time).toNSec()*1e-6;
				r.sleep();
				if (last_time_stamp == 0) // hasnt published yet
				{
					d_msg.level = diagnostic_msgs::DiagnosticStatus::STALE;
					d_msg.message = "Haven't published yet";
					ROS_WARN_STREAM(imu_name << ": "<< d_msg.message);
					diagnostic_msgs::DiagnosticArray a_diags_msg;
					a_diags_msg.status.push_back(d_msg);
					da_msg = a_diags_msg;
					//
				}
				if (execution_time > 100) // hasnt updated timestamp
				{
					d_msg.level = diagnostic_msgs::DiagnosticStatus::STALE; // maybe error?
					d_msg.message = "Timestamp hasn't changed. Is the IMU ON? execution_time is " + std::to_string(execution_time) + "[ms]";
					ROS_WARN_STREAM(imu_name << ": "<< d_msg.message);
					diagnostic_msgs::DiagnosticArray a_diags_msg;
					a_diags_msg.status.push_back(d_msg);
					da_msg = a_diags_msg;
					//
				}
				else if (da_msg.status.size() <1){
					d_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
					std::stringstream s;
					s <<  std::fixed << std::setprecision(2) << "\tBat.(" << battery_percentage_monitor << ")\tDelay.("<< time_delay_monitor*1000 << "[ms])";
					d_msg.message = s.str();
					diagnostic_msgs::DiagnosticArray a_diags_msg;
					a_diags_msg.status.push_back(d_msg);
					da_msg = a_diags_msg;
				}

				da_msg.header.stamp = ros::Time::now();
				diags_pub.publish(da_msg);
				da_msg.status.clear();
			}

			connection->close();
		}

	private:
		sensor_msgs::Imu imu_msg;

		std::function<void(ximu3::XIMU3_DecodeError error)> decodeErrorCallback = [](auto decode_error)
		{
			ROS_ERROR_STREAM(XIMU3_decode_error_to_string(decode_error) );
		};

		std::function<void(ximu3::XIMU3_Statistics statistics)> statisticsCallback = [](auto statistics)
		{
			/*printf(TIMESTAMP_FORMAT UINT64_FORMAT " bytes" UINT32_FORMAT " bytes/s" UINT64_FORMAT " messages" UINT32_FORMAT " messages/s" UINT64_FORMAT " errors" UINT32_FORMAT " errors/s\n",
			  statistics.timestamp,
			  statistics.data_total,
			  statistics.data_rate,
			  statistics.message_total,
			  statistics.message_rate,
			  statistics.error_total,
			  statistics.error_rate);*/
			ROS_DEBUG_STREAM( XIMU3_statistics_to_string(statistics)); // alternative to above
		};

		std::function<void(ximu3::XIMU3_InertialMessage message)> inertialCallback = [this](auto message)
		{
			if (false)
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT " deg/s" FLOAT_FORMAT " deg/s" FLOAT_FORMAT " deg/s" FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
					message.timestamp,
					message.gyroscope_x,
					message.gyroscope_y,
					message.gyroscope_z,
					message.accelerometer_x,
					message.accelerometer_y,
					message.accelerometer_z);
			// std::cout << XIMU3_inertial_message_to_string(message) << std::endl; // alternative to above
		
			imu_msg.angular_velocity.x = message.gyroscope_x;
			imu_msg.angular_velocity.y = message.gyroscope_y;
			imu_msg.angular_velocity.z = message.gyroscope_z;
			

			imu_msg.linear_acceleration.x = message.accelerometer_x;
			imu_msg.linear_acceleration.y = message.accelerometer_y;
			imu_msg.linear_acceleration.z = message.accelerometer_z;
			
		};

		std::function<void(ximu3::XIMU3_MagnetometerMessage message)> magnetometerCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT " a.u." FLOAT_FORMAT " a.u." FLOAT_FORMAT " a.u.\n",
					message.timestamp,
					message.x_axis,
					message.y_axis,
					message.z_axis);
			// std::cout << XIMU3_magnetometer_message_to_string(message) << std::endl; // alternative to above
		};

		ros::WallTime get_estimated_imu_time(uint64_t message_time)
		{
		//TODO: doing this using uint64_ts and converting to int32_t for WallDuration was too hard for me, so we are just going to live with an implementation using double. I hope I am not adding too much noise here. Fix this if you want to deal with carries and other things.- maybe there is an easier way with creating an empty duration and then setting it with fromNsecs, well, too late now. 

		double k = 1e-6; // ????
		auto current_time = ros::WallTime::now();
		ros::WallTime res;
		//std::cout << "message_time: " << message_time << std::endl;
		if (initial_time_stamp == 0 || current_time == initial_time) // in case the elapsed time was zero, to avoid a divide by zero 
		{
			initial_time_stamp = message_time; 
			initial_time = ros::WallTime::now();
			res = initial_time; 
		}
		else // we initialized the initial time, so now we need to make a simple conversion
		{
		
			double elapsed_time = k * (message_time - initial_time_stamp); // in ns
			//std::cout << "I might be wrong: " << elapsed_time << "[s]"  << std::endl;

			ros::WallTime complete_time = initial_time + ros::WallDuration(elapsed_time);

			res = complete_time;

		}


		return res;
		}


		std::function<void(ximu3::XIMU3_QuaternionMessage message)> quaternionCallback = [this](auto message)
		{
			std::chrono::time_point c1= std::chrono::high_resolution_clock::now();
			ros::WallTime this_time = ros::WallTime::now();
			double execution_time = (this_time - last_time).toNSec()*1e-6;
			last_time = this_time;
			//ROS_DEBUG_STREAM("Elapsed time (ms): " << execution_time << " | Rate: " << 1/execution_time*1000);

			//geometry_msgs::PoseStamped pp;

			//printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT "\n",
			//       message.timestamp,
			//       message.w_element,
			//       message.x_element,
			//       message.y_element,
			//       message.z_element);
			// std::cout << XIMU3_quaternion_message_to_string(message) << std::endl; // alternative to above
			//tf2::Quaternion myQuaternion(message.x_element, message.y_element,message.z_element, message.w_element);
			//geometry_msgs::Quaternion quat_msg = tf2::toMsg(myQuaternion);

			//pp.header.frame_id = "torax";
			//pp.header.stamp = ros::Time::now();
			//pp.pose.orientation = quat_msg;
			//poser_pub.publish(pp);
			this_time_stamp = message.timestamp;
			if (last_time_stamp == 0)
				last_time_stamp = this_time_stamp; // to initialize the thing and not give an absurd amount of time delay on first duration period
								   //ROS_INFO_STREAM(this_time_stamp); //microssecond

								   //the attempt to correct receiving 4 messages at the same time, however sometimes it messes it up. so removing for now!
								   //auto fake_time = ros::Time::now() + ros::Duration((double)(this_time_stamp - last_time_stamp)/1000000);
			auto estimate_time = get_estimated_imu_time(this_time_stamp);

			time_delay_monitor = this_time.toSec() - estimate_time.toSec();
			geometry_msgs::TransformStamped transformStamped;
			ros::Time some_time;
			if (use_imu_time_stamps)
				some_time.fromNSec(estimate_time.toNSec());
			else
				some_time = ros::Time::now();
			transformStamped.header.stamp = some_time;
			//transformStamped.header.stamp = fake_time;
			/*transformStamped.header.frame_id = "map";
			  transformStamped.child_frame_id = "ximu3";*/
			transformStamped.header.frame_id = parent_frame_id;
			transformStamped.child_frame_id = child_frame_id;

			//ROS_INFO_STREAM("time now" << transformStamped.header.stamp << " estimated_time" << estimate_time << "difference: " << (transformStamped.header.stamp.toSec() - estimate_time.toSec()) << "[s] ");

			if (do_calibration && !q_cal)
			{  // TODO: test if this is actually correct, and that is a big if. but if it is, then we can probably make this function into a service call, 
			   // make it receive some tf, maybe you can specify it by name
			   // and then this will calibrate the imu to have a new "starting" point, as in, it could be calibrated in place, say with an ar marker
				ROS_WARN_STREAM("WARNING: EXPERIMENTAL! You may want to just fix the Yaw here, instead of the whole inverse tf...");

				ROS_WARN_STREAM("CALIBRATING QUATERNION!!" <<XIMU3_quaternion_message_to_string(message) );
				q_cal = tf2::Quaternion{message.x_element, message.y_element, message.z_element, message.w_element};
			}
			tf2::Quaternion r{message.x_element, message.y_element, message.z_element, message.w_element};
			/*ROS_INFO_STREAM("current q_cal: " 
			  << q_cal.getX() << " , "
			  << q_cal.getY() << " , " 
			  << q_cal.getZ() << " , " 
			  << q_cal.getW() );*/

			//this seems incorrect. 
			transformStamped.transform.rotation = tf2::toMsg(r*q_cal->inverse()); // the one that looks most like the correct thing so far
											      //transformStamped.transform.rotation = tf2::toMsg(q_cal->inverse()*r);

											      //transformStamped.transform.rotation = tf2::toMsg(r.inverse());
											      //transformStamped.transform.rotation = tf2::toMsg(r**q_cal);

											      //
											      //transformStamped.transform.rotation = tf2::toMsg(*q_cal*r*q_cal->inverse());
			ROS_DEBUG_STREAM(transformStamped.transform.rotation);
			if (false)
			{
				transformStamped.transform.rotation.x = message.x_element;
				transformStamped.transform.rotation.y = message.y_element;
				transformStamped.transform.rotation.z = message.z_element;
				transformStamped.transform.rotation.w = message.w_element;
			}
			//	    transformStamped.transform.setOrigin(tf2::Vector3(0.5, 0.0, 0.0) );
			transformStamped.transform.translation.x = origin[0];	
			transformStamped.transform.translation.y = origin[1];	
			transformStamped.transform.translation.z = origin[2];	

			br.sendTransform(transformStamped);

			imu_msg.header = transformStamped.header;
			imu_msg.orientation = transformStamped.transform.rotation;
			imu_pub.publish(imu_msg);
			ros::spinOnce();
			std::chrono::time_point c2= std::chrono::high_resolution_clock::now();
			//ROS_INFO_STREAM("loop time: "<< std::chrono::duration_cast<std::chrono::microseconds>(c2-c1).count()<<"[us]");

		};

		std::function<void(ximu3::XIMU3_RotationMatrixMessage message)> rotationMatrixCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT "\n",
					message.timestamp,
					message.xx_element,
					message.xy_element,
					message.xz_element,
					message.yx_element,
					message.yy_element,
					message.yz_element,
					message.zx_element,
					message.zy_element,
					message.zz_element);
			// std::cout << XIMU3_rotation_matrix_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_EulerAnglesMessage message)> eulerAnglesCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT " deg" FLOAT_FORMAT " deg" FLOAT_FORMAT " deg\n",
					message.timestamp,
					message.roll,
					message.pitch,
					message.yaw);
			// std::cout << XIMU3_euler_angles_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_LinearAccelerationMessage message)> linearAccelerationCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
					message.timestamp,
					message.w_element,
					message.x_element,
					message.y_element,
					message.z_element,
					message.x_axis,
					message.y_axis,
					message.z_axis);
			// std::cout << XIMU3_linear_acceleration_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_EarthAccelerationMessage message)> earthAccelerationCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
					message.timestamp,
					message.w_element,
					message.x_element,
					message.y_element,
					message.z_element,
					message.x_axis,
					message.y_axis,
					message.z_axis);
			// std::cout << XIMU3_earth_acceleration_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_HighGAccelerometerMessage message)> highGAccelerometerCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
					message.timestamp,
					message.x_axis,
					message.y_axis,
					message.z_axis);
			// std::cout << XIMU3_high_g_accelerometer_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_TemperatureMessage message)> temperatureCallback = [this](auto message)
		{
			//printf(TIMESTAMP_FORMAT FLOAT_FORMAT " degC\n",
			//       message.timestamp,
			//       message.temperature);
			ROS_DEBUG_STREAM( XIMU3_temperature_message_to_string(message)); // alternative to above
			std_msgs::Float32 temperature;
			if (message.temperature > 60)
			{
				d_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
				da_msg.status.push_back(d_msg);
				da_msg.header.stamp = ros::Time::now();
				diags_pub.publish(da_msg);
				ROS_FATAL("IMU TEMPERATURE TOO HIGH!!");
				throw std::runtime_error("Temperature of IMU over 60C. Shutting off!!!");
				std::exit(EXIT_FAILURE);
			}

			if (message.temperature > 50)
			{
				d_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
				da_msg.status.push_back(d_msg);
				ROS_ERROR_STREAM("IMU has temperature of more than 50C!!!");
			}

			temperature.data = message.temperature;
			temp_pub.publish(temperature);
			ros::spinOnce();
		};

		std::function<void(ximu3::XIMU3_BatteryMessage message)> batteryCallback = [this](auto message)
		{
			//printf(TIMESTAMP_FORMAT FLOAT_FORMAT " %%" FLOAT_FORMAT " V" FLOAT_FORMAT "\n",
			//       message.timestamp,
			//       message.percentage,
			//       message.voltage,
			//       message.charging_status);
			std_msgs::Float32 battery_percentage,battery_voltage;
			battery_percentage.data = message.percentage;
			battery_percentage_monitor = message.percentage;
			battery_voltage.data = message.voltage;
			bat_pub.publish(battery_percentage);

			if(message.percentage < 30)
			{
				ROS_WARN_STREAM("Battery about to run out on IMU: " << imu_name);
				d_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
				da_msg.status.push_back(d_msg);
			}
			bat_v_pub.publish(battery_voltage);

			ROS_DEBUG_STREAM( XIMU3_battery_message_to_string(message)); // alternative to above
			ros::spinOnce();
		};

		std::function<void(ximu3::XIMU3_RssiMessage message)> rssiCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT FLOAT_FORMAT " %%" FLOAT_FORMAT " dBm\n",
					message.timestamp,
					message.percentage,
					message.power);
			// std::cout << XIMU3_rssi_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_SerialAccessoryMessage message)> serialAccessoryCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT STRING_FORMAT "\n",
					message.timestamp,
					message.char_array);
			// std::cout << XIMU3_serial_accessory_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_NotificationMessage message)> notificationCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT STRING_FORMAT "\n",
					message.timestamp,
					message.char_array);
			// std::cout << XIMU3_notification_message_to_string(message) << std::endl; // alternative to above
		};

		std::function<void(ximu3::XIMU3_ErrorMessage message)> errorCallback = [](auto message)
		{
			printf(TIMESTAMP_FORMAT STRING_FORMAT "\n",
					message.timestamp,
					message.char_array);
			// std::cout << XIMU3_error_message_to_string(message) << std::endl; // alternative to above
		};
};
