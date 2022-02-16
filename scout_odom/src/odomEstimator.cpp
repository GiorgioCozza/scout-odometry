



#include "ros/ros.h"
#include <math.h>
#include <time.h>

#include <geometry_msgs/TwistStamped.h>
#include "scout_odom/odomEstimate.h"
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>

#include "scout_odom/odomPoseInit.h"
#include "scout_odom/odomPoseReset.h"

#include "scout_odom/odomParamsConfig.h"
#include <dynamic_reconfigure/server.h>


#define _DEBUG_MODE


class odomEstimator {
	
	public:

		odomEstimator(){

			odest_msg.odom.pose.pose.position.x = 0.0;
			odest_msg.odom.pose.pose.position.y = 0.0;
			odest_msg.odom.pose.pose.position.z = 0.0;
			odest_msg.odom.pose.pose.orientation.x = 0.0;
			odest_msg.odom.pose.pose.orientation.y = 0.0;
			odest_msg.odom.pose.pose.orientation.z = 0.0;
			odest_msg.odom.pose.pose.orientation.w = 1.0;
			
			t_init = false;

			x = 0.0;
			y = 0.0;
			theta = 0.0;
			t = 0.0;

			lst_x = 0.0;
			lst_y = 0.0;
			lst_theta = 0.0;
			lst_t = 0.0;

			sub = nh.subscribe("/scout_kin", 1000, &odomEstimator::update_odometry_cb, this);

			estod_pub = nh.advertise<scout_odom::odomEstimate>("estim_odom", 1000);
			odom_pub = nh.advertise<nav_msgs::Odometry>("estim_odom_odom", 1000);

			pose_init_srv = nh.advertiseService("odom_pose_init", &odomEstimator::pose_init, this);
			pose_reset_srv = nh.advertiseService("odom_pose_reset", &odomEstimator::pose_reset, this);

			f = boost::bind(&odomEstimator::dc_callback, this, _1, _2);
			server.setCallback(f);
		}

		virtual ~odomEstimator(){ }


		void dc_callback(scout_odom::odomParamsConfig &conf, uint32_t bitmsk){


			char * im_text;

			if (conf.int_method == 0)
				im_text = "euler";
			else
				im_text = "rk";

			#ifdef _DEBUG_MODE
			ROS_INFO("\n\t[ LOG odomEstimator ] Initialization of Scout 2.0 at position: \ 
				\n\n\t\t - X:    %f \
				\n\t\t - Y:    %f \
				\n\t\t - Z:    %f \
				\n using %s method \n", conf.x_init, conf.y_init, conf.theta_init, im_text);
			#endif

			t = 0.0;
			x = conf.x_init;
			y = conf.y_init;
			theta = conf.theta_init;
			if (conf.int_method == odomEstimator::EULER)
				int_met =  odomEstimator::EULER;
			else
				int_met =  odomEstimator::RUNGE_KUTTA;

		}


		void update_odometry_cb(const geometry_msgs::TwistStampedConstPtr& ts){

			double t_sec = ts->header.stamp.sec;
			double t_nsec = ts->header.stamp.nsec * pow(10.0, -9.0);

			t = t_sec + t_nsec;

			// takes first step to synchronize time && get rid of nan values 
			if ( t - lst_t > 1.0 ) {
			
			}else {

				double delta_t = t - lst_t;
				
				if (int_met == odomEstimator::EULER){

					x = lst_x + ( ts->twist.linear.x * delta_t * cos(lst_theta) );
					y = lst_y + ( ts->twist.linear.x * delta_t * sin(lst_theta) );
					theta = lst_theta + ( ts->twist.angular.z * delta_t );
					std_msgs::String mth_msg;
					mth_msg.data = "euler";
					odest_msg.method = mth_msg;

				} else if (int_met == odomEstimator::RUNGE_KUTTA){

					x = lst_x + ts->twist.linear.x * delta_t * cos(lst_theta + ( ts->twist.angular.z * delta_t / 2 ));
					y = lst_y + ts->twist.linear.x * delta_t * sin(lst_theta + ( ts->twist.angular.z * delta_t / 2 ));
					theta = lst_theta + ( ts->twist.angular.z * delta_t );
					std_msgs::String mth_msg;
					mth_msg.data = "rk";
					odest_msg.method = mth_msg;
				}

				tf2::Quaternion qt;
				qt.setRPY(0.0, 0.0, theta);
				qt = qt.normalize();

				odest_msg.odom.header = ts->header;
				odest_msg.odom.header.frame_id = "odom";
				odest_msg.odom.child_frame_id = "base_link";
				odest_msg.odom.twist.twist = ts->twist;
				odest_msg.odom.pose.pose.position.x = x;
				odest_msg.odom.pose.pose.position.y = y;
				odest_msg.odom.pose.pose.position.z = 0.0;
				odest_msg.odom.pose.pose.orientation.x = qt[0];
				odest_msg.odom.pose.pose.orientation.y = qt[1];
				odest_msg.odom.pose.pose.orientation.z = qt[2];
				odest_msg.odom.pose.pose.orientation.w = qt[3];

				estod_pub.publish(odest_msg);
				odom_pub.publish(odest_msg.odom);
			}


			lst_t = t;
			lst_x = x;
			lst_y = y;
			lst_theta = theta;

		}

		bool pose_init(scout_odom::odomPoseInit::Request &req, scout_odom::odomPoseInit::Response &res){

			x = req.x_init;
			y = req.y_init;
			theta = req.theta_init;

			lst_x = req.x_init;
			lst_y = req.y_init;
			lst_theta = req.theta_init;
			// need to re-sync time
			t_init = false;

			#ifdef _DEBUG_MODE
			ROS_INFO("\n\t[ LOG odomEstimator ] Initial position modified: \n \
			  \n\n\t\t - X_INIT :         %f \
				\n\t\t - Y_INIT :         %f \
				\n\t\t - THETA_INIT :     %f\n", req.x_init,
											 req.y_init,
											 req.theta_init );
			#endif

			return true;
		}


		bool pose_reset(scout_odom::odomPoseReset::Request &req, scout_odom::odomPoseReset::Response &res){

			x = 0.0;
			y = 0.0;
			theta = 0.0;

			lst_x = 0.0;
			lst_y = 0.0;
			lst_theta = 0.0;

			// need to re-sync time
			t_init = false;

			#ifdef _DEBUG_MODE
			ROS_INFO("\n\t[ LOG odomEstimator ] Initial position modified: \n \
				\n\n\t\t - X_INIT :         %f \
				\n\t\t - Y_INIT :         %f \
				\n\t\t - THETA_INIT :     %f\n", 0.0,
											 0.0,
											 0.0 );
			#endif

			return true;
		}


	private:

		ros::NodeHandle nh;

		double lst_t;
		double lst_x;
		double lst_y;
		double lst_theta;

		double t;
		double x;
		double y;
		double theta;

		bool t_init;

		enum method { EULER = 0, RUNGE_KUTTA = 1 } int_met;
		scout_odom::odomEstimate odest_msg;
		nav_msgs::Odometry odom_msg;
		ros::Publisher estod_pub;
		ros::Publisher odom_pub;
		ros::Subscriber sub;
		
		dynamic_reconfigure::Server<scout_odom::odomParamsConfig> server;
		dynamic_reconfigure::Server<scout_odom::odomParamsConfig>::CallbackType f;

		ros::ServiceServer pose_init_srv;
		ros::ServiceServer pose_reset_srv;
};


int main(int argc, char **argv){


	ros::init(argc, argv, "odom_estimator");

	odomEstimator odest;

	ros::spin();

	return 0;
}