
#ifndef CTRL_IO_H
#define CTRL_IO_H

// number of micro-switches
#define NB_U_SWITCH 2

// number of stored rising/falling edges
#define NB_STORE_EDGE 10

// ID of the right and left sides
enum
{
	R_ID,
	L_ID
};

/// Controller inputs
typedef struct CtrlIn
{
	/*! //brief time reference
	 */
	double t; ///< time [s]

	/*! //brief wheel speeds
	 *
	 * Each wheel speed is positive when the robot is going forward (the front part of the robot is
	 * the round part).
	 */
	double r_wheel_speed; ///< right wheel speed [rad/s]
	double l_wheel_speed; ///< left wheel speed [rad/s]
	double r_wheel_ref;
	double l_wheel_ref;

	//Odometry variables

	double r_odo_dist;
	double r_odo_dist_prev;
	double l_odo_dist;
	double l_odo_dist_prev;

	/*! //brief micro-switches
	 */
	int u_switch[NB_U_SWITCH]; ///< 1 if corresponding u_switch (R_ID or L_ID) is activated, 0 otherwise

	//array for the proximity sensors
	double sens_array_front[5]; // [0] left side [1-3] front [5] right side
	double sens_array_back[5];	// [0] left side [1-3] front [5] right side
	int sens_flag;
    //                                      Front
	//array for the color sensors     L  -----------  R
	double color_array_right[3];//    0  |         |  0
	double color_array_left[3];//     1  |         |  1
	//                                2  |         |  2
	//                                   ----------- 	
    //                                       Back
#ifdef SIMU_PROJECT
		/*! brief tower for the fixed beacon
	 *
	 * These variables are similar to the same ones without the '_fixed' name, except that
	 * they only detect the fixed beacon placed at the border of the map.
	 * Only the beacon of the corresponding team are detected.
	 */
		double last_rising_fixed[NB_STORE_EDGE]; ///< rotating list with the last rising edges detected [rad]
	double last_falling_fixed[NB_STORE_EDGE];	 ///< rotating list with the last falling edges detected [rad]

	int rising_index_fixed;	 ///< index in 'last_rising' of the last element added
	int falling_index_fixed; ///< index in 'last_falling' of the last element added

	int nb_rising_fixed;  ///< number of rising edges detected during the last laser revolution
	int nb_falling_fixed; ///< number of falling edges detected during the last laser revolution

	/*! \brief targets
	 */
	int nb_targets;		 ///< number of targets carried by the robot
	int target_detected; ///< 1 if target currently detected under the robot, 0 otherwise

	/*! \brief joystick-keyboard
	 */
	int keyboard_arrow[2][2];	  ///< arrows keyboard or (Z,Q,S,D / W,A,S,D) (signals in the range [-100;100], see user_realtime_events.cc)
	double joystick_handle[4][2]; ///< joystick handle (signals in the range [-1;1])

	int keyboard_key[2];	///< keyboard keys (space bar and enter key, see user_realtime_events.cc)
	int joystick_button[4]; ///< joystick buttons (depend on the joystick)

	/*! brief robot ID
	 *
	 * These are the following robots IDs (with their corresponding teams):
	 *    ROBOT_B (blue)  : 0 - team A
	 *    ROBOT_R (red)   : 1 - team A
	 *    ROBOT_Y (yellow): 2 - team B
	 *    ROBOT_W (white) : 3 - team B
	 */
	int robot_id; ///< ID of the robot
#endif

} CtrlIn;

/// Controller outputs
typedef struct CtrlOut
{
	/*! \brief wheel commands
	 *
	 * Each wheel is commanded by a voltage in the range [-24 V ; +24 V]. To avoid duty cycles problems, it is better
	 * to work in the range [-0.9*24 V ; +0.9*24 V].
	 *
	 * 'wheel_commands' is a command signal bounded in [-100 ; 100], proportional to the voltage sent to the corresponding
	 * wheel motor. Here are three examples of voltages corresponding to 'wheel_commands' values:
	 *   -100 corresponds to -0.9*24 V
	 *      0 corresponds to       0 V
	 *    100 corresponds to +0.9*24 V
	 */
	double wheel_commands[2]; ///< wheel motors (R_ID or L_ID) commands [-], bounded in [-100 ; 100]

#ifdef SIMU_PROJECT
	/*! \brief targets release
	 *
	 * When this flag is set to 1, the robot automatically releases all the targets it is carrying.
	 * Consequently, the robot cannot pick any target when this flag is set to 1.
	 * Set this flag to 0 to carry targets.
	 */
	int flag_release;
#endif

} CtrlOut;

#endif
