


#include "ros/ros.h"
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include "robotics_hw1/MotorSpeed.h"

#include "scout_odom/calibParamsConfig.h"

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "scout_odom/calibInput.h"
#include <nav_msgs/Odometry.h>
#include "scout_odom/calibStats.h"

#define _DEBUG_MODE

class kinematicsPub {
	

public:
	kinematicsPub() : msfr_sub(nh, "motor_speed_fr", 1),
					  msfl_sub(nh, "motor_speed_fl", 1),
					  msrr_sub(nh, "motor_speed_rr", 1),
					  msrl_sub(nh, "motor_speed_rl", 1),
				  	  msSync(msfr_sub, msfl_sub, msrr_sub, msrl_sub, 10)
					  {

		vr = 0.0;
	    vl = 0.0;

	    cbin.vr = 0.0;
	    cbin.vl = 0.0;

		calib_input_pub = nh.advertise<scout_odom::calibInput>("/calib_input", 1000);

		msSync.registerCallback(boost::bind(&kinematicsPub::motor_speed_filter_cb, this, _1, _2, _3, _4));

		gf = boost::bind(&kinematicsPub::calib_params_cb, this, _1, _2);
		calib_server.setCallback(gf);

	}


	virtual ~kinematicsPub(){}


	void motor_speed_filter_cb ( const robotics_hw1::MotorSpeed::ConstPtr& msfr,
					             const robotics_hw1::MotorSpeed::ConstPtr& msfl,
					             const robotics_hw1::MotorSpeed::ConstPtr& msrr,
					             const robotics_hw1::MotorSpeed::ConstPtr& msrl

					        ){

		std_msgs::Header head = msfr->header;
		cbin.header = head;

		double ms_frw = msfr->rpm;
		double ms_flw = -msfl->rpm;
		double ms_rrw = msrr->rpm;
		double ms_rlw = -msrl->rpm;
	    
		// x [rad/s] = 60*y/M_PI [rpm]
		double wang_speeds[4] = {(M_PI*ms_frw)/30, (M_PI*ms_flw)/30,
							     (M_PI*ms_rrw)/30, (M_PI*ms_rlw)/30};

		double wtg_speeds[4] = { wang_speeds[0]*wheel_radius, wang_speeds[1]*wheel_radius,
							     wang_speeds[2]*wheel_radius, wang_speeds[3]*wheel_radius };

		
		// skid-steering assumption v_fr = v_rr (v_fl = v_rl), mean considered: (v_fr + v_rr)/2

		vr = (wtg_speeds[0] + wtg_speeds[2])/2.0;
		vl = (wtg_speeds[1] + wtg_speeds[3])/2.0;

		cbin.vr = vr;
		cbin.vl = vl;

		calib_input_pub.publish(cbin);

	}

	void calib_params_cb(scout_odom::calibParamsConfig &conf, uint32_t bitmsk){


		wheel_radius = conf.wheel_radius;

		#ifdef _DEBUG_MODE
		ROS_INFO("\n\t[ LOG kinematicsPub ] New geometry configs: \
						  \n\n\t\t - WHEEL RADIUS :    %f \
						  \n\t\t - GEAR_RATIO_MAX :    %f \
						  \n\t\t - GEAR_RATIO_MIN :    %f \n", conf.wheel_radius, 
						                                  conf.gear_ratio_max,
						                                  conf.gear_ratio_min);
		#endif
	}


private:

	double wheel_radius;
	double apparent_baseline;
	double gear_ratio;

	double vr;
	double vl;

	ros::NodeHandle nh;

	ros::Publisher calib_input_pub;

	message_filters::Subscriber<robotics_hw1::MotorSpeed> msfr_sub;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> msfl_sub;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> msrr_sub;
	message_filters::Subscriber<robotics_hw1::MotorSpeed> msrl_sub;

	message_filters::TimeSynchronizer<robotics_hw1::MotorSpeed,
					  				  robotics_hw1::MotorSpeed,
					  				  robotics_hw1::MotorSpeed,
					  				  robotics_hw1::MotorSpeed> msSync;

	scout_odom::calibInput cbin;

	dynamic_reconfigure::Server<scout_odom::calibParamsConfig> calib_server;
	dynamic_reconfigure::Server<scout_odom::calibParamsConfig>::CallbackType gf;
};


int main(int argc, char **argv){
	
	ros::init(argc, argv, "kinematicsPub");

	kinematicsPub kinpub;

	ros::spin();

	return 0;
}