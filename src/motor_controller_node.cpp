#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
//#include "phidgets_api/motor.h"
//#include "phidgets_api/phidget.h"
#include <stdlib.h>
#include <math.h>
#include "phidget22.h"

#include <string.h>

#include <sstream>

#define FREQUENCY 100 //Hz  Execution frequency of the programm

#define VINT_SN 620709

#define AXLE_WIDTH 0.375 // [m]
#define V_LIN_MAX 1.5  // [m/s]
#define V_ANG_MAX 2.0  // [rad/s]
#define A_Y_MAX 2.5    // [m/s²]

#define TIRE_RADIUS 0.08                             // [m]
#define TIRE_CIRCUMFERENCE (TIRE_RADIUS * 2.0 * M_PI) // [m]

#define NOMINAL_MOTOR_SPEED 3.0333 // [1/s]
#define NOMINAL_MOTOR_VOLTAGE 24.0 // [V]
#define GEAR_RATIO 18              // 18:1 [INPUT:OUTPUT]
#define ENCODER_RESOLUTION 300     //[Counts/Revolution] (on the engine side of the gearbox)

#define KP 5 // I dont know what im doing, should probably be changed!
#define MOTOR_ACCELERATION 1.0 // Duty cycles/s [0.1:100]
#define MOTOR_BRAKING_STRENGTH 1.0 //

#define MOTOR_CURRENT_LIMIT 25  // [A]  [2:250]
#define MOTOR_DATA_INTERVALL  100  //data interval in ms

typedef struct
{
  ros::Time t_old;
  int64_t pos_old;
  PhidgetEncoderHandle handle;
} encoder_t;

enum controller_side_t
{
  left = 0x00,
  right = 0x01
};

bool assert_driving_command();

void set_duty_cycle(PhidgetDCMotorHandle *handle, double duty_cycle);

void calculate_duty_cycle(double *duty_cycle, double setpoint, double actual_value, double battery_voltage);

void get_speed(encoder_t *encoder, double *v_actual);

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg);

bool connect_controller(PhidgetDCMotorHandle *ch, controller_side_t side);

bool connect_temp_sens(PhidgetTemperatureSensorHandle *ch);

double calculate_r(double v_lin, double v_ang);

void calculate_v(geometry_msgs::Twist cmd_vel, double *v_l, double *v_r);

void report_device_info(PhidgetHandle handle);

void connect_motor_controllers(PhidgetDCMotorHandle **mc_array);
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

geometry_msgs::Twist cmd_vel;

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "motor_controller_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmd_vel_cb);

  ros::Rate loop_rate(FREQUENCY);

  PhidgetDCMotorHandle ch;

  // Connect the motor controllers
  PhidgetDCMotorHandle mc_handle_1;
  PhidgetDCMotorHandle mc_handle_2;

  PhidgetDCMotorHandle *mc_handles[2];

  mc_handles[left] = &mc_handle_1;
  mc_handles[right] = &mc_handle_2;

  connect_motor_controllers(mc_handles);

  PhidgetTemperatureSensorHandle ch_tmp;

  if (connect_temp_sens(&ch_tmp))
  {
    double temp;
    PhidgetTemperatureSensor_getTemperature(ch_tmp, &temp);
    ROS_INFO("Temperatur: %g", temp);
  }

  std_msgs::String msg;

  double v_l_setpoint;
  double v_r_setpoint;
  double v_l_actual;
  double v_r_actual;
  double dc_l;
  double dc_r;

  encoder_t encoder_l;
  encoder_t encoder_r;

  encoder_l.t_old = ros::Time::now();
  encoder_l.pos_old = 0;

  encoder_r.t_old = ros::Time::now();
  encoder_r.pos_old = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    if (assert_driving_command())
    {
      calculate_v(cmd_vel, &v_l_setpoint, &v_r_setpoint); // calculate the setpoint speeds for both wheels

      get_speed(&encoder_l, &v_l_actual); // caculate the actual speed of the wheels
      get_speed(&encoder_r, &v_r_actual);

      calculate_duty_cycle(&dc_l, v_l_setpoint, v_l_actual, 24); // calculate the duty cycle using a simple P controller
      calculate_duty_cycle(&dc_r, v_r_setpoint, v_r_actual, 24); // with a feedforward component

      set_duty_cycle(mc_handles[left], dc_l); // apply the calculated duty cycle values
      set_duty_cycle(mc_handles[right], dc_r);
    }

    //PhidgetDCMotor_setTargetVelocity(ch, cmd_vel.linear.x);

    //ROS_INFO("%s", msg.data.c_str());
    double temp;
    PhidgetTemperatureSensor_getTemperature(ch_tmp, &temp);
    // ROS_INFO("Temperatur: %g", temp);
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  PhidgetDCMotor_delete(&ch);

  return 0;
}

bool assert_driving_command()
{
  return true;
}

void connect_motor_controllers(PhidgetDCMotorHandle **mc_array)
{
  connect_controller(mc_array[left], left);
  connect_controller(mc_array[right], right);

  report_device_info((PhidgetHandle)*mc_array[left]);
  report_device_info((PhidgetHandle)*mc_array[right]);
}

void set_duty_cycle(PhidgetDCMotorHandle *handle, double duty_cycle)
{
  PhidgetDCMotor_setTargetVelocity(*handle, duty_cycle);
}

void calculate_duty_cycle(double *duty_cycle, double setpoint, double actual_value, double battery_voltage)
{
  // Feedforward
  //  Calculating the estimated duty cycle needed for the requested engine speed.
  double dc_ff = setpoint / (battery_voltage * NOMINAL_MOTOR_SPEED * TIRE_CIRCUMFERENCE / NOMINAL_MOTOR_VOLTAGE);

  // Feedback loop
  //  A basic P controller that corrects the error from the engine load
  double error = setpoint - actual_value;
  double dc_fb = KP * error;

  // adding feedforward and feedback loop
  *duty_cycle = dc_ff + dc_fb;

  // Saturate output
  if (*duty_cycle > 1)
  {
    *duty_cycle = 1;
  }
  if (*duty_cycle < -1)
  {
    *duty_cycle = -1;
  }
}

void get_speed(encoder_t *encoder, double *v_actual)
{
  int64_t position = 0;
  PhidgetEncoder_getPosition(encoder->handle, &position);
  ros::Time now = ros::Time::now();

  int64_t d_pos = position - encoder->pos_old;
  ros::Duration diff_time = now - encoder->t_old;

  double d_time = diff_time.toSec();

  *v_actual = ((d_pos / d_time) / (ENCODER_RESOLUTION * GEAR_RATIO)) * TIRE_CIRCUMFERENCE;

  encoder->pos_old = position;
  encoder->t_old = now;
}

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_vel = *msg;
}

bool connect_controller(PhidgetDCMotorHandle *ch, controller_side_t side)
{
  PhidgetReturnCode res;

  PhidgetDCMotor_create(ch);

  Phidget_setDeviceSerialNumber((PhidgetHandle)*ch, VINT_SN);
  Phidget_setHubPort((PhidgetHandle)*ch, side);

  ROS_INFO("Connecting to device...");

  res = Phidget_openWaitForAttachment((PhidgetHandle)*ch, 1000);
  if (res != EPHIDGET_OK)
  {
    ROS_INFO("Connection Failed; code: 0x%x", res); // Exit in error
    return false;
  }
  else
  {
    ROS_INFO("Connected");
    PhidgetDCMotor_setTargetBrakingStrength(*ch, MOTOR_BRAKING_STRENGTH);
    PhidgetDCMotor_setAcceleration(*ch, MOTOR_ACCELERATION);
    PhidgetDCMotor_setDataInterval(*ch, MOTOR_DATA_INTERVALL);
    PhidgetDCMotor_setCurrentLimit(*ch, MOTOR_CURRENT_LIMIT);
    return true;
  }
}

void report_device_info(PhidgetHandle handle)
{ 
  const char *name;
  int32_t deviceID;
  int32_t SN;
  int32_t port;

  Phidget_getDeviceName(handle, &name);
  Phidget_getDeviceSerialNumber(handle, &SN);
  Phidget_getHubPort(handle, &port);

  ROS_INFO("device %s connected to port %d (VINT SN: %d) ", name, port, SN);
}

bool connect_temp_sens(PhidgetTemperatureSensorHandle *ch)
{

  PhidgetReturnCode res;

  PhidgetTemperatureSensor_create(ch);

  ROS_INFO("Connecting to the temperature sensor");

  res = Phidget_openWaitForAttachment((PhidgetHandle)*ch, PHIDGET_TIMEOUT_DEFAULT);
  if (res != EPHIDGET_OK)
  {
    ROS_INFO("Connection to the temp sensor failed; error 0x%x", res);
    return false;
  }
  else
  {
    ROS_INFO("Connection to the temp sensor successful");
    return true;
  }
}

double calculate_r(double v_lin, double v_ang)
{

  // calculate velocity difference left - right
  double d_v_ang = v_ang * AXLE_WIDTH; // delta v in [m/s]

  // superposition of v_lin and v_ang
  double v_l = v_lin + d_v_ang / 2;
  double v_r = v_lin - d_v_ang / 2;

  return (AXLE_WIDTH / 2) * (v_r + v_l) / (v_r - v_l);
}

void calculate_v(geometry_msgs::Twist cmd_vel, double *v_l_setpoint, double *v_r_setpoint)
{

  double v_lin, v_ang, v_l, v_r;
  uint8_t state = 0;

  if (cmd_vel.linear.x != 0)
  {
    state = state + 0x01;

    if (abs(cmd_vel.linear.x) > V_LIN_MAX)
    { // Limit maximum linear speed
      if (cmd_vel.linear.x > 0)
      {
        v_lin = V_LIN_MAX;
      }
      else
      {
        v_lin = -V_LIN_MAX;
      }
      ROS_INFO("Linear speed exceeds limits -> Reduced speed from %g to %g!", cmd_vel.linear.x, v_lin);
    }
    else
    {
      v_lin = cmd_vel.linear.x;
    }
  }

  if (cmd_vel.angular.z != 0)
  {
    state = state + 0x02;

    if (abs(cmd_vel.angular.z) > V_ANG_MAX)
    { // Limit maximum angular speed
      if (cmd_vel.angular.z > 0)
      {
        v_ang = V_ANG_MAX;
      }
      else
      {
        v_ang = -V_ANG_MAX;
      }
      ROS_INFO("Angular speed exceeds limits -> Reduced speed from %g to %g!", cmd_vel.angular.z, v_ang);
    }
    else
    {
      v_ang = cmd_vel.angular.z;
    }
  }

  switch (state)
  {

  case 0: // v_lin and v_ang = 0 -> dont move
    v_l = 0;
    v_r = 0;
    break;

  case 1: // v_ang = 0 -> only linear motion
    v_l = v_lin;
    v_r = v_lin;
    break;

  case 2: // v_lin = 0 -> only angular motion
  {
    double d_v_ang = v_ang * AXLE_WIDTH; // delta v in [m/s]

    v_l = d_v_ang / 2;
    v_r = -d_v_ang / 2;
  }
  break;

  case 3: // v_lin and v_ang != 0 -> make sure the robot doesnt tip over
  {
    double r = calculate_r(v_lin, v_ang);

    if (abs(v_lin * v_lin / r) > A_Y_MAX)
    { // calculate the lateral acceleration
      if (v_lin > 0)
      {
        v_lin = sqrt(abs(r) * A_Y_MAX); // lateral acceleration to high -> reduce speed
      }
      else
      {
        v_lin = -sqrt(abs(r) * A_Y_MAX); // lateral acceleration to high -> reduce speed
      }
      ROS_INFO("lateral acceleration exceeds limits -> Reduced speed!");
    }

    ROS_INFO("Radius: %g, speed: %g", r, v_lin);

    v_l = v_lin * (r - 0.5 * AXLE_WIDTH) / r; // Calculate the wheel speeds (may need to switch l and r)
    v_r = v_lin * (r + 0.5 * AXLE_WIDTH) / r;
  }
  break;

  default:
    ROS_ERROR("Somehow reached invalid state in the motor controller node");
    break;
  }

  *v_l_setpoint = v_l;
  *v_r_setpoint = v_r;
  // convert to rounds per second
  // *rps_l = v_l / TIRE_CIRCUMFERENCE;
  // *rps_r = v_r / TIRE_CIRCUMFERENCE;

  ROS_INFO("v_l: %g, v_r: %g", *v_l_setpoint, *v_r_setpoint);
  ROS_INFO(" ");
}
