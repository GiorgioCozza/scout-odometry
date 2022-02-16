



#include "ros/ros.h"
#include <math.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>

#include "scout_odom/odomEstimate.h"
#include "scout_odom/estimStats.h"
#include <nav_msgs/Odometry.h>

#include "scout_odom/rvizParamsConfig.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/PoseStamped.h>
#include "scout_odom/odomPoseInit.h"
#include "scout_odom/odomPoseReset.h"


#define _DEBUG_MODE

class rviz_prod {
	


	public:
	rviz_prod() : estim_odom_sub(nh, "/estim_odom_odom", 1),
				  scout_odom_sub(nh, "/scout_odom", 1),
				  rvizSync(estim_odom_sub, scout_odom_sub, 10)
				  {


		init_done = false;

		n = 0;
		cum_so_x = 0.0;
		cum_so_y = 0.0;
		cum_so_theta = 0.0;
		cum_eo_x = 0.0;
		cum_eo_y = 0.0;
		cum_eo_theta = 0.0;

		es.pose_x_rmse = 0.0;
		es.pose_y_rmse = 0.0;
		es.pose_theta_rmse = 0.0;

		estim_odom_pub = nh.advertise<nav_msgs::Odometry>("/rviz_estim_odom", 1000);
		scout_odom_pub = nh.advertise<nav_msgs::Odometry>("/rviz_scout_odom", 1000);
		estim_stats_pub = nh.advertise<scout_odom::estimStats>("/estim_stats", 1000);

		rvizSync.registerCallback(boost::bind(&rviz_prod::rviz_filter_cb, this, _1, _2));

		init_client = nh.serviceClient<scout_odom::odomPoseInit>("odom_pose_init");

		rviz_f = boost::bind(&rviz_prod::rviz_dc_callback, this, _1, _2);
		rviz_server.setCallback(rviz_f);

	}

	virtual ~rviz_prod(){}



	void rviz_filter_cb ( const nav_msgs::Odometry::ConstPtr& estim_odom,
						  const nav_msgs::Odometry::ConstPtr& scout_odom ){


		nav_msgs::Odometry eo;
		nav_msgs::Odometry so;

		// Initialization of pose to the scout_odom to highlight accuracy
		if (!init_done){

			tf2::Quaternion qt(scout_odom->pose.pose.orientation.x,
							   scout_odom->pose.pose.orientation.y,
							   scout_odom->pose.pose.orientation.z,
							   scout_odom->pose.pose.orientation.w);

			tf2::Matrix3x3 mat3(qt);

			double roll, pitch, yaw;
			mat3.getRPY(roll, pitch, yaw);

			// Alignment to scout_odom
			pose_init_srv.request.x_init = scout_odom->pose.pose.position.x;
			pose_init_srv.request.y_init = scout_odom->pose.pose.position.y;
			pose_init_srv.request.theta_init = yaw;

			if (init_client.call(pose_init_srv)){

				#ifdef _DEBUG_MODE
				ROS_INFO("\n\t[ LOG rviz_prod ] Request for alignment with bag odometry");
				#endif
				init_done = true;
			} else {

				#ifdef _DEBUG_MODE
				ROS_ERROR("\n\t[ LOG rviz_prod ] Error during the alignment phase");
				#endif
			}
			// estim_stats reset
			n = 0;
			cum_so_x = 0.0;
			cum_so_y = 0.0;
			cum_so_theta = 0.0;
			cum_eo_x = 0.0;
			cum_eo_y = 0.0;
			cum_eo_theta = 0.0;

			es.pose_x_rmse = 0.0;
			es.pose_y_rmse = 0.0;
			es.pose_theta_rmse = 0.0;
		}
		

		eo.header = estim_odom->header;
		eo.header.stamp = ros::Time::now();
		eo.twist = estim_odom->twist;
		eo.pose = estim_odom->pose;

		so.header = scout_odom->header;
		so.header.stamp = ros::Time::now();
		so.twist = scout_odom->twist;
		so.pose = scout_odom->pose;

		es.stamp = ros::Time::now();
		update_estim_stats(estim_odom, scout_odom);

		estim_odom_pub.publish(eo);
		scout_odom_pub.publish(so);
		estim_stats_pub.publish(es);



	}


	void update_estim_stats( const nav_msgs::Odometry::ConstPtr& estim_odom, 
							 const nav_msgs::Odometry::ConstPtr& scout_odom ){

			n++;
			cum_so_x += scout_odom->pose.pose.position.x;
			cum_so_y += scout_odom->pose.pose.position.y;

			cum_eo_x += estim_odom->pose.pose.position.x;
			cum_eo_y += estim_odom->pose.pose.position.y;

			// conversion Quaternions to Euler angles 
			tf2::Quaternion qt_so(scout_odom->pose.pose.orientation.x,
							      scout_odom->pose.pose.orientation.y,
							      scout_odom->pose.pose.orientation.z,
							      scout_odom->pose.pose.orientation.w);


			tf2::Quaternion qt_eo(estim_odom->pose.pose.orientation.x,
							      estim_odom->pose.pose.orientation.y,
							      estim_odom->pose.pose.orientation.z,
							      estim_odom->pose.pose.orientation.w);

			tf2::Matrix3x3 mat3_so(qt_so);
			tf2::Matrix3x3 mat3_eo(qt_eo);

			double roll_so, pitch_so, yaw_so;
			double roll_eo, pitch_eo, yaw_eo;

			mat3_so.getRPY(roll_so, pitch_so, yaw_so);
			mat3_eo.getRPY(roll_eo, pitch_eo, yaw_eo);

			cum_so_theta += yaw_so;
			cum_eo_theta += yaw_eo;

			es.pose_x_rmse = sqrt(pow(cum_so_x - cum_eo_x, 2)/n);
			es.pose_y_rmse = sqrt(pow(cum_so_y - cum_eo_y, 2)/n);
			es.pose_theta_rmse = sqrt(pow(cum_so_theta - cum_eo_theta, 2)/n);



	}

	void rviz_dc_callback(scout_odom::rvizParamsConfig &conf, uint32_t bitmsk){


		init_done = conf.init_done;

		#ifdef _DEBUG_MODE
		ROS_INFO("\n\t[ LOG rviz_prod ] New rviz_prod configs: \
						  \n\n\t\t - INIT_DONE :                 %d", 	conf.init_done);
		#endif 

	}


	private:

	bool init_done;

	unsigned int n;
	double cum_so_x;
	double cum_so_y;
	double cum_so_theta;
	double cum_eo_x;
	double cum_eo_y;
	double cum_eo_theta;

	ros::NodeHandle nh;

	ros::Publisher estim_odom_pub;
	ros::Publisher scout_odom_pub;
	ros::Publisher estim_stats_pub;

	ros::ServiceClient init_client;
	scout_odom::odomPoseInit pose_init_srv;

	scout_odom::estimStats es;

	message_filters::Subscriber<nav_msgs::Odometry> estim_odom_sub;
	message_filters::Subscriber<nav_msgs::Odometry> scout_odom_sub;
	
	message_filters::TimeSynchronizer<nav_msgs::Odometry, 
								  	  nav_msgs::Odometry> rvizSync;

	dynamic_reconfigure::Server<scout_odom::rvizParamsConfig> rviz_server;
	dynamic_reconfigure::Server<scout_odom::rvizParamsConfig>::CallbackType rviz_f;

};


int main(int argc, char **argv){

	ros::init(argc, argv, "rviz_prod");

	rviz_prod rp;

	ros::spin();

	return 0;
}