// C++ includes
//#include <Eigen/Dense>

#include "assisted_joypad_flying/assisted_joypad_flying.h"


namespace joypad_node
{

JoypadFlying::JoypadFlying():
    nh_(),
    desired_state_(),
    copilot_feedback_msg_(),
    publish_position_(false)
{
  joypad_ = sensor_msgs::Joy();
  joypad_.axes = std::vector<float>(8,0);
  joypad_.buttons = std::vector<int32_t>(8,0);

  reloadParameters();

  state_estimate_sub_ = nh_.subscribe("state_estimate", 10, &JoypadFlying::receiveStateEstimateCallback, this);
  joypad_sub_ = nh_.subscribe("joy",10, &JoypadFlying::joyCallback, this);
  copilot_feedback_sub_ = nh_.subscribe("copilot/feedback",10, &JoypadFlying::copilotFeedbackCallback, this);
  assisted_commands_sub_= nh_.subscribe("assisted_commands",10,&JoypadFlying::receiveAssistedCommandCallback,this);



  desired_state_pub_ = nh_.advertise< quad_msgs::QuadDesiredState >("copilot/desired_state", 10);
  start_pub_ = nh_.advertise< std_msgs::Empty >("copilot/start",1);
  land_pub_ = nh_.advertise< std_msgs::Empty >("copilot/land",1);
  //feedthrough_pub_ = nh_.advertise< std_msgs::Bool >("copilot/feedthrough",1);

  looptimer_ = nh_.createTimer(ros::Duration( 0.1 ), &JoypadFlying::mainloop, this);
}

JoypadFlying::~JoypadFlying()
{
}

void JoypadFlying::receiveStateEstimateCallback(const quad_msgs::QuadStateEstimate::ConstPtr& msg)
{
  state_estimate_ = QuadState( *msg );
}

void JoypadFlying::copilotFeedbackCallback(const quad_msgs::ControllerFeedbackConstPtr &msg)
{
  copilot_feedback_msg_ = * msg;
}

void JoypadFlying::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  joypad_ = *joy_msg;
  time_last_joypad_msg_ = ros::Time::now();
}

void JoypadFlying::receiveAssistedCommandCallback(const forest_msgs::output_commands::ConstPtr &msg)
{
    assisted_speed=msg->speed;
    assisted_relative_yaw=msg->relativeYaw;
    assisted_delta_z=msg->deltaZ;
    time_last_joypad_msg_ = ros::Time::now();
 }


bool JoypadFlying::estimateAvailable()
{
  if( state_estimate_.estimator_id <= 0 )
    { return false; }
  if( ( ros::Time::now() - state_estimate_.timestamp).toSec() > 0.2)
    {return false; }
  return true;
}

bool JoypadFlying::copilotInHover()
{
  if( copilot_feedback_msg_.state == copilot::states::HOVER &&
      ( ros::Time::now() - copilot_feedback_msg_.header.stamp ).toSec() < 0.2  )
  {
    return true;
  }
  return false;
}

bool JoypadFlying::mustUpdateDesiredState()
{
  static int prev_state = copilot::states::INVALID;
  if( copilot_feedback_msg_.state != prev_state )
  {
    prev_state = copilot_feedback_msg_.state;
    if(copilotInHover())
    {
      return true;
    }
  }
  return false;
}

void JoypadFlying::mainloop(const ros::TimerEvent& time)
{
  static int packetctr=0;

  if( estimateAvailable() && copilotInHover() && (ros::Time::now() - time_last_joypad_msg_) < ros::Duration(joypad_timeout_))
  {
    publish_position_ = true;
  }
  else
  {
    publish_position_ = false;
  }

  // update the desired state if the state of the copilot changed to hover
  if( mustUpdateDesiredState() )
  {
    ROS_INFO("[%s] updating the ref position", ros::this_node::getName().c_str() );
    desired_state_ = QuadDesiredState();
    desired_state_.position = state_estimate_.position;
    desired_state_.yaw = quaternionToEulerAnglesZYX(state_estimate_.orientation).z();
  }

  //
  // publish a desired state update if in manual hover mode
  //
  if( publish_position_ )
    {

      //desired_state feeds into a PI(position)D(velocity) controller at
      //controllers/hover_angle_rate_controller/src/hover_controller.cpp

      //settings desired_state_.position.x/y would trigger the P and I components too.
      
      double alpha_velocity = 1 - exp(-looprate_ / tau_velocity_);
      
      double desired_speed=vmax_xy_ * joypad_.axes[axes::SPEED] + joypad_.axes[axes::SPEED_ASSISTED]*assisted_speed;
      desired_state_.yaw += (rmax_yaw_ * joypad_.axes[axes::YAW] + joypad_.axes[axes::YAW_ASSISTED]*assisted_relative_yaw) * looprate_ ;
      desired_state_.yaw =  wrapMinusPiToPi( desired_state_.yaw );  
      
      QuadDesiredState measured_state;
      
      measured_state.velocity.x() = cos(desired_state_.yaw)*desired_speed;
      measured_state.velocity.y() = sin(desired_state_.yaw)*desired_speed;
      measured_state.velocity.z() = vmax_z_  * joypad_.axes[axes::Z] + joypad_.axes[axes::SPEED_ASSISTED]*assisted_delta_z;

      desired_state_.velocity = (1.0 - alpha_velocity) * desired_state_.velocity + alpha_velocity * measured_state.velocity ;
      desired_state_.position += desired_state_.velocity * looprate_;

      if( flyingroom_.useBounds() )
      {
        flyingroom_.forceWithinArena( desired_state_ );
      }

      quad_msgs::QuadDesiredState desired_state_msg;
      desired_state_msg.header.stamp = ros::Time::now();
      desired_state_msg.header.seq = packetctr++;
      desired_state_msg.position = eigenToGeometry( desired_state_.position );
      desired_state_msg.velocity = eigenToGeometry( desired_state_.velocity );
      desired_state_msg.yaw = desired_state_.yaw;

      desired_state_pub_.publish( desired_state_msg );
    }
    //
    // Start and Land Buttons
    //
    if( joypad_.buttons[buttons::GREEN] )
    {
      start_pub_.publish( std_msgs::Empty() );
    }
    if( joypad_.buttons[buttons::BLUE] )
    {
      land_pub_.publish( std_msgs::Empty() );
    }
}

bool JoypadFlying::reloadParameters( )
{
  std::string node_name = ros::this_node::getName();
  ROS_INFO( "[%s] Updating parameters", node_name.c_str() );

  //if( !loadRosParameter(node_name, "joypad_scale_speed", joypad_scale_speed_ ) ) return false;
  //if( !loadRosParameter(node_name, "joypad_scale_z", joypad_scale_z_ ) ) return false;
  //if( !loadRosParameter(node_name, "joypad_scale_yaw", joypad_scale_yaw_ ) ) return false;

  if( !getParam("joypad_timeout", joypad_timeout_ ) ) return false;
  if( !getParam("vmax_xy", vmax_xy_ ) ) return false;
  if( !getParam("vmax_z", vmax_z_ ) ) return false;
  if( !getParam("rmax_yaw", rmax_yaw_ ) ) return false;
  if( !getParam("looprate", looprate_ ) ) return false;
  if( !getParam("tau_velocity", tau_velocity_ ) ) return false;
  

  return true;
}

} /* namespace joypad_flying */


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joypad");
  joypad_node::JoypadFlying joypad_flying;
  ros::spin();
  return 0;
}

