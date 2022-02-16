
TEAM MEMBER INFO

Person Code :		10649461
Matricola   : 		904965
Email		:       giorgio.cozza@mail.polimi.it
Name	    : 		Giorgio
Surname	    : 		Cozza

_______________________________________________________________


ACRONYMS (also used in the code)

abl	 : apparent baseline
gr 	 : gear ratio
gr_avg 	 : average value of estimate gear ratio
abl_avg  : average value of estimated apparent baseline
lst_*	 : last value of *
cum_* 	 : cumulative value of *

_______________________________________________________________


SOURCE FILES DESCRIPTION

The project folder contains all the source files required to
estimate the odometry, in particular:

- kinematicsPub.cpp : implementing a message filter to synchronize
		      the velocity of the 4 wheel motors. From 
		      such info Vr and Vl are computed (without 
		      considering the gear ratio)

- calibrator.cpp    : includes a message filter to synchronize
		      the scout_odom messages(manufacturer odometry)  
		      and the messages published by kinematicsPub
		      which are used to calibrate the baseline and
		      the gear ratio and then use both to compute
		      the robot kinematics.

- odomEstimator.cpp : implements the algorithm for the odometry
		      estimation with the possibility to switch
		      between Euler and Runge-Kutta integration
		      methods.

- rviz_prod.cpp     : synchronizes and publishes the messages 
		      it received (scout_odom and estim_odom_odom)
		      and in particular this node is used to 
		      prepare data for visualization with RVIZ


OTHER FILES/FOLDERS

rviz (folder) : contains the configuration file for rviz. 
				This is provided as argument in the launch file
				to start rviz together with all the nodes.

_______________________________________________________________


PARAMETER DESCRIPTION

PRIMARY: 

1) (src/scout_odom/cfg/odomParams.cfg)

- x_init 	: Initial position on x axis (const double_t)
- y_init 	: Initial position on y axis (const double_t)
- theta_init 	: Initial orientation w.r.t z axis (Yaw) (const double_t)

- int_method	: List of available integration methods in the 
		  algorithm ( enum )


SECONDARY : 

2) (src/scout_odom/cfg/calibParams.cfg)

DESC: Set of parameters including geometry and kinematic
      information about the robot wheels and motors 
      and other values required during calibration phase


- conv_steps	 	: number of consecutive steps during offline 
					  calibration in which the root square error
					  between the average and its last value (lst_avg)
					  must be less than the error tolerance value
					  (for both abl and gr) (int_t)

- gr_error_tolerance    : max RSE between gr_avg and its last vaue 
						  accepted as convergence condition for 
						  gear ratio (double_t)

- abl_error_tolerance	: max RSE between abl_avg and its last vaue 
						  accepted as convergence condition for 
						  apparent baseline (double_t)

- default_abl		: starting value of the apparent baseline
					  in case of calibration already carried out
					  default value corresponds to the calibrated
					  value (double_t)

- default_gr 		: starting value of the gear ratio
					  in case of calibration already carried out
					  default value corresponds to the calibrated
					  value (double_t)

- online_calib		: flag parameter that enables online calibration
					  In this mode, the apparent baseline and the 
					  gear ratio are updated step by step with 
		          		the computed values (bool_t)

- calib_done		: flag parameter that tells if calibration is 
		          	  performed and provided the best results.
					  in case this flag is set to True the 
					  default values are taken into account
					  as calibrated values (bool_t)


- wheel_radius		: wheel radius (double_t)

- gear_ratio_min	: min gear ratio by specs (double_t)

- gear_ratio_max	: max gear ratio by specs (double_t)

- real_baseline		: real baseline (double_t)

3) (src/scout_odom/rvizParams.cfg)

- init_done : boolean value to set to "false" every time a new bag is 
		 	  executed, it allows to rviz_prod to align 
		 	  scout_odom pose with estim_odom pose 
_______________________________________________________________


CUSTOM MESSAGES DESCRIPTION

1)
src/scout_odom/msg/calibInput.msg

Defines a message containing the input for the calibrator

vr : velocity of right-side 
vl : velocity of left-side

both does not consider the gear ratio

2)
src/scout_odom/msg/calibStats.msg

Defines a message to monitor estimation of apparent baseline
and gear ratio, the average values of the estimations computed
so far and the root square error of the average values w.r.t
to its previous value.

3)
src/scout_odom/msg/estimStats.msg

Defines a message to monitor the performance of the estimated
odometry in terms of the RMSE of (x,y,theta) w.r.t the scout_odom 
(considered as ground-truth pose)

4)
src/scout_odom/msg/odomEstimate.msg

Defines the message containing the estimated odometry
in particular:

- nav_msgs/Odometry odom     (the computed odometry)
- std_msgs/String method     (integration method)

_______________________________________________________________

HOW TO START/USE THE NODES

ASSUMPTION: All changes in parameters and crucial steps in
		    program execution are notified to the output. 
		    If it is annoying is possible avoid this condition
		    by commenting the _DEBUG_MODE macro in all the node 
			sources and recompile DON'T DO IT FOR calibrator.cpp.
			The calibrated apparent baseline and gear ratio will
			be printed out by this part. 
			

1: Start all the nodes by using the launch file in launch folder,
	project1.launch that will also start RVIZ with the configuration file
	provided in rviz folder.

2: Before starting the bag play, some parameters
   may be changed to affect the calibration procedure. 
   Defaul values however appear to be good for testing.

   NOTE: by the messages read on Teams and from lecture
	 it turned out that the apparent baseline of the differential
	 drive approx in a skid steering robot should be
	 ideally constant and this is also valid for the gear ratio
	 but in ideal conditions. 
	 This expects to have an offline calibration in which
	 both the apparent baseline and the gear ratio are
	 computed and the corresponding values are then use 
	 to estimated the odometry. However in my calibration 
	 procedure I also included a mode by which is possible
	 to estimate both the abl and gr online (obtaining at each
	 step different values). It is not clear how much correct
	 is this practice. It is possible to change this mode 
	 by setting the parameter "online_calib" to 0.

    2.a: IMPORTANT: In normal condition the calibration mode is offline. 
         At each step the apparent baseline and gear ratio is evaluated 
         as follows: 

         abl = ((vr - vl)*gear_ratio)/(scout_odom_twist_angular_z)

         gr = 2 * scout_odom_twist_linear_x / (vr + vl)

         The average value of both is updated at each step, and if its RSE
	 stays in a tolerance range (abl_error_tolerance and gr_error_tolerance) 
	 for a consecutive number of steps (conv_steps) the calibration
	 ends and the odometry estimation starts.

    	 so before starting it is possible (but not mandatory) to set 
    	 the following parameters:

    	 - conv_steps : a smaller value increases the speed of convergence
    	 		        but it provides less stable values
    	 
    	 - abl_error_tolerance : an higher value speeds up the convergence but 
			     	provides more uncertain estimation
    	 
    	 - gr_error_tolerance : (equivalent to abl_error_tolerance for gear_ratio)

    	 - default_abl : starting value for apparent baseline. If the
			 calibrated value is available it is possible to 
			 set such parameter at the beginning together with 
			 "calib_done" to true in such a way 
			 the calibration is avoided and it starts the odometry
			 estimation phase.

    	 - default_gr :  (equivalent to default_abl for gear ratio)

    	 - calib_done : (used when calibrated values are available)

    Default values produce good results in acceptable time.


3: Open 3 shells and run:

	s1: rostopic echo -c /calib_stats

	s2: rostopic echo -c /estim_stats

	s3: rostopic echo -c /estim_odom

to monitor calibration performance and odometry value and performance at runtime.


4: Run the bag1 until the calibration is completed by using:

	rosbag play --clock --pause bag1.bag

   then start the play. After that, the values will be printed
   out on the shell that launched the launch file.

   Interrupt or let the bag1 running until the end, after
   calibration the estim_odom and scout_odom will be displayed on
   rviz. From now on, calib_stats is not published, while instead it 
   is estim_stats.
   Before playing the remaining 2 bags (withouth stopping the nodes),
   run:

   rosrun dynamic_reconfigure dynparam set /rvizProd init_done false

   to perform alignment of estim_odom to scout_odom pose, then run the bag2.
   Repeat same procedure for bag3.

   DETAILS: Once finished calibration rviz_prod calls the
	  service to set the initial pose of estimated odometry
	  to the one of the scout_odom to appreciate the
	  trend of the estimation w.r.t the ground-truth (scout_odom).
	  This is also used to compute the mean root square error
	  for (x,y,theta)


   WARNING 4.1 : After calibration if there are misalignments or the estim_odom is 
   not visible (anomaly in odometry estimation), try to stop the bag 
   set the /rvizProd init_done to "false" and re-run the bag. 
   If it does not work try to reset the init pose. If even this last steategy 
   does not solve the problem last solution is to restart setting "online_calib"
   to "true" that always provided positive results.

   WARNING 4.2 : It is possible to test the odom_pose_init and odom_pose_reset
		services during odometry computation but this will 
		compromise the evaluation of the estim_stats










END.

