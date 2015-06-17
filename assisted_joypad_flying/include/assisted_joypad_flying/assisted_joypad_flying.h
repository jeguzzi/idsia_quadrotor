/*
 * JoypadFlying.h
 *
 *  Created on: Feb 13, 2014
 *      Author: ffontana
 */

#ifndef JOYPADFLYING_H_
#define JOYPADFLYING_H_

#include <Eigen/Dense>

#include "ros/ros.h"

#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"

#include "quad_msgs/QuadStateEstimate.h"
#include "quad_msgs/HoverControllerFeedback.h"
#include "quad_msgs/QuadDesiredState.h"

#include "quad_common/geometry_eigen_conversions.h"
#include "quad_common/quad_desired_state.h"
#include "quad_common/math_common.h"
#include "quad_common/quad_state.h"
#include "quad_common/parameter_helper.h"

#include "flyingroom/flyingroom.h"

#include "copilot/states.h"
#include "assisted_joypad_flying/joypad_buttons_axes.h"

#include "forest_msgs/output_commands.h"

namespace joypad_node
{


using namespace quad_common;

class JoypadFlying
{
public:
  JoypadFlying();
  virtual ~JoypadFlying();

private:
  void receiveStateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr& msg);
  void copilotFeedbackCallback(const quad_msgs::HoverControllerFeedbackConstPtr &msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void receiveAssistedCommandCallback(const forest_msgs::output_commands::ConstPtr& msg);

  bool estimateAvailable();
  bool mustUpdateDesiredState();
  bool copilotInHover();

  bool reloadParameters( );

  void mainloop(const ros::TimerEvent& time);

  ros::NodeHandle nh_;

  ros::Publisher desired_state_pub_;
  ros::Publisher start_pub_;
  ros::Publisher land_pub_;
  ros::Publisher feedthrough_pub_;


  ros::Subscriber state_estimate_sub_;
  ros::Subscriber joypad_sub_;
  ros::Subscriber copilot_feedback_sub_;
  ros::Subscriber assisted_commands_sub_;


  ros::Timer looptimer_;

  sensor_msgs::Joy joypad_;
  quad_msgs::HoverControllerFeedback copilot_feedback_msg_;

  QuadState state_estimate_;
  QuadDesiredState desired_state_;

  bool publish_position_;

  flyingroom::Flyingroom flyingroom_;

  double joypad_scale_xy_;
  double joypad_scale_speed_;
  double joypad_scale_z_;
  double joypad_scale_yaw_;

  double assisted_relative_yaw;
  double assisted_speed;
  double assisted_delta_z;
};

} /* namespace joypad_flying */

#endif /* JOYPADFLYING_H_ */
