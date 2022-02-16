



#include <math.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <scout_odom/calibParamsConfig.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include "scout_odom/calibStats.h"
#include "scout_odom/calibInput.h"

#define _DEBUG_MODE


class calibrator {


public:
	calibrator() : calib_sub(nh, "calib_input", 1),
				   odom_sub(nh, "scout_odom", 1),
				   calibSync(calib_sub, odom_sub, 10)
				   {

	    vl = 0.0;
	    vr = 0.0;

    	apparent_baseline = 0.0;
		gear_ratio = 0.0;
		calib_done = false;

		cum_abl = 0.0;
		cum_gr = 0.0;
		lst_avg_abl = 0.0;
		lst_avg_gr = 0.0;
		nsmp_abl = 0;
		nsmp_gr = 0;
		remain_steps = 0;
		rse_tol_gr = 0.0;
		rse_tol_abl = 0.0;

		cg.apparent_baseline_estim = 0.0;
		cg.gear_ratio_estim = 0.0;
		cg.abl_avg = 0.0;
		cg.gr_avg = 0.0;
		cg.abl_avg_rse = 0.0;
		cg.gr_avg_rse = 0.0;

		calibStats_pub = nh.advertise<scout_odom::calibStats>("calib_stats", 1000);
		calibTwist_pub = nh.advertise<geometry_msgs::TwistStamped>("scout_kin", 1000);
		calibSync.registerCallback(boost::bind(&calibrator::calib_pub, this, _1, _2));

		cf = boost::bind(&calibrator::calib_params_cb, this, _1, _2);

		calib_server.setCallback(cf);

	}

	virtual ~calibrator(){}




	void calib_pub( const scout_odom::calibInput::ConstPtr& cbin,
					const nav_msgs::Odometry::ConstPtr& odmsg ){

		double abl, gr;
		
		vr = cbin->vr;
		vl = cbin->vl;

		if (!calib_done || online_calib){

			// avoid nan/inf values
			if (odmsg->twist.twist.angular.z != 0 && vr != vl){

				// calibration of apparent baseline
				abl = ((vr - vl)*gear_ratio)/(odmsg->twist.twist.angular.z);

				// update stats if apparent baseline in range
				 if ( abl >= real_baseline && abl < 1.1 ){

				 	 nsmp_abl++;
					 cum_abl += abl;
					 cg.abl_avg = cum_abl / nsmp_abl;
					 cg.abl_avg_rse = sqrt(pow(cg.abl_avg - lst_avg_abl,2));

					 cg.apparent_baseline_estim = abl;

					 if (online_calib)
					 	apparent_baseline = abl;

				}

			}


			// avoid nan/inf values
			if ( odmsg->twist.twist.linear.x != 0 && vr + vl != 0 ){

				// calibration of gear ratio
				gr = 2 * odmsg->twist.twist.linear.x / (vr + vl);
				
				// update stats if gear_ration in range
				if ( gr >= gear_ratio_min && gr <= gear_ratio_max ){
					nsmp_gr++;
					cum_gr += gr;
					cg.gr_avg = cum_gr / nsmp_gr;
					cg.gr_avg_rse = sqrt(pow(cg.gr_avg - lst_avg_gr,2));

					cg.gear_ratio_estim = gr;

					if (online_calib)
						gear_ratio = gr;
				}

				
			}


			// under convergence conditions (e.g: average rse < tolerance) update calibrated values
			// and decrement remaining steps 
			if ( cg.abl_avg_rse <= rse_tol_abl && cg.gr_avg_rse <= rse_tol_gr && 
				 cg.abl_avg_rse > 0.0  &&  cg.gr_avg_rse > 0.0 ){
				 	
					
					apparent_baseline = cg.abl_avg;
				 	gear_ratio = cg.gr_avg;
				 	remain_steps--;
			 
 			// otherwise use estimated values if non-zero
			} else {
				
			 	remain_steps = conv_steps;
			}

			lst_avg_abl = cg.abl_avg;
			lst_avg_gr = cg.gr_avg;
			
			if ( remain_steps == 0 && !online_calib ){

				calib_done = true;

				#ifdef _DEBUG_MODE
				ROS_INFO("\n\t[ LOG calibrator ] Calibration performed: \
								  \n\n\t\t - APPARENT_BASELINE :    %f \
								  \n\t\t - GEAR_RATIO :           %f", apparent_baseline, 
						                                        gear_ratio);
				#endif
			
			}

			if (online_calib){
				scout_kinematics_pub(odmsg->header);
			}

			cg.stamp = cbin->header.stamp;
			calibStats_pub.publish(cg);

		} else {
			// compute kinematics and publish
			scout_kinematics_pub(odmsg->header);
		}
	}


	void scout_kinematics_pub(std_msgs::Header header){

		// compute kinematics
		ts.header = header;
		ts.twist.linear.x = (vr + vl)*gear_ratio/2.0;
		ts.twist.linear.y = 0.0;
		ts.twist.linear.z = 0.0;
		ts.twist.angular.x = 0.0;
		ts.twist.angular.y = 0.0;
		ts.twist.angular.z = (vr - vl)*gear_ratio/apparent_baseline;

		calibTwist_pub.publish(ts);

	}


	void calib_params_cb( scout_odom::calibParamsConfig& conf, uint32_t bitmask ){

		conv_steps = conf.conv_steps;
		remain_steps = conv_steps;
		rse_tol_abl = conf.abl_error_tolerance;
		rse_tol_gr = conf.gr_error_tolerance;
		online_calib = conf.online_calib;

		if (online_calib)
			calib_done = false;
		else
			calib_done = conf.calib_done;

		apparent_baseline = conf.default_abl;
		gear_ratio = conf.default_gr;
		
		gear_ratio_min = conf.gear_ratio_min;
		gear_ratio_max = conf.gear_ratio_max;

		real_baseline = conf.real_baseline;

		#ifdef _DEBUG_MODE
		ROS_INFO("\n\t[ LOG calibrator ] New calibration configs: \
						  \n\n\t\t - CONV_STEPS :                 %d \
						  \n\t\t - ABL_ERROR_TOLERANCE :        %f \
						  \n\t\t - GR_ERROR_TOLERANCE :         %f \
						  \n\t\t - DEFAULT_BASELINE :           %f \
						  \n\t\t - DEFAULT_GEAR_RATIO :         %f \
						  \n\t\t - CALIB_DONE :                 %d \
						  \n\t\t - ONLINE_CALIB :               %d \
						  \n\t\t - WHEEL_RADIUS :               %f \
						  \n\t\t - GEAR_RATIO_MIN :             %f \
						  \n\t\t - GEAR_RATIO_MAX :             %f \
						  \n\t\t - REAL_BASELINE :              %f\n", 	conf.conv_steps, 
										                                conf.abl_error_tolerance,
										                                conf.gr_error_tolerance,
										                                conf.default_abl,
										                                conf.default_gr,
										                                calib_done,
										                                conf.online_calib,
										                                conf.wheel_radius, 
										                                conf.gear_ratio_min,
										                                conf.gear_ratio_max,
										                                conf.real_baseline);
		#endif 

	}


private:
	double vl;
	double vr;

	// calibration parameters
	unsigned int conv_steps;
	double rse_tol_gr;
	double rse_tol_abl;
	double real_baseline;
	double gear_ratio_min;
	double gear_ratio_max;
	bool online_calib;

	// calibration results
	double apparent_baseline;
	double gear_ratio;

	// calibration stats
	double cum_abl;
	double cum_gr;
	double lst_avg_abl;
	double lst_avg_gr;
	unsigned int nsmp_abl;
	unsigned int nsmp_gr;
	unsigned int remain_steps;
	bool calib_done;

	scout_odom::calibStats cg;
	geometry_msgs::TwistStamped ts;

	ros::NodeHandle nh;
	ros::Publisher calibStats_pub;
	ros::Publisher calibTwist_pub;

	message_filters::Subscriber<scout_odom::calibInput> calib_sub;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub;

	message_filters::TimeSynchronizer<scout_odom::calibInput,
									  nav_msgs::Odometry> calibSync;

	dynamic_reconfigure::Server<scout_odom::calibParamsConfig> calib_server;

	dynamic_reconfigure::Server<scout_odom::calibParamsConfig>::CallbackType cf;


};


int main(int argc, char **argv){

	ros::init(argc, argv, "calibrator");

	calibrator calib;

	ros::spin();

	return 0;

}