#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Temperature.h"
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
#define V_LIN_MAX 1.5    // [m/s]
#define V_ANG_MAX 2.0    // [rad/s]
#define A_Y_MAX 2.5      // [m/s²]

#define TIRE_RADIUS 0.08                              // [m]
#define TIRE_CIRCUMFERENCE (TIRE_RADIUS * 2.0 * M_PI) // [m]

#define NOMINAL_MOTOR_SPEED 3.0333 // [1/s]
#define NOMINAL_MOTOR_VOLTAGE 24.0 // [V]
#define GEAR_RATIO 18              // 18:1 [INPUT:OUTPUT]
#define ENCODER_RESOLUTION 1200    //[Counts/Revolution] (on the engine side of the gearbox)
#define MOTOR_ACCELERATION 1.0     // Duty cycles/s [0.1:100]
#define MOTOR_BRAKING_STRENGTH 1.0 // Braking strength if the set velocity is 0 [0:1.0]

#define MOTOR_CURRENT_LIMIT 25     // [A]  [2:25]
#define MOTOR_DATA_INTERVALL 100   //data interval in ms [100:60000]
#define ENCODER_DATA_INTERVALL 100 //data interval in ms [100:60000]

#define POSITION_CONTROLLER_DATA_INTERVAL 20 // [ms]
#define POSITION_CONTROLLER_FAILSAFE_TIMER 500 // [ms]
#define POSITION_CONTROLLER_CURRENT_REGULATOR_GAIN 10 //
//#define POSITION_CONTROLLER_RESCALE_FACTOR 5400 // encoder steps per revolution

#define K_P 0.5 // P component of the PID     I dont know what im doing, should probably be changed!
#define K_I 1  // I component of the PID
#define K_D 0  // D component of the PID

typedef struct
{
  double dt;
  double integral;
  double last_error;
} mc_pid_t;

enum controller_side_t
{
  left = 0x00,
  right = 0x01
};

enum mc_state_t
{
  init = 0x00,
  connect_controllers,
  wait_for_rtd,
  drive,
  brake,
  disengaged,
  deinit

}

typedef struct
{
  ros::Publisher temp_l;
  ros::Publisher temp_r;
  ros::Publisher current_l;
  ros::Publisher current_r;
  ros::Publisher speed_l;
  ros::Publisher speed_r;
} ros_publisher_t;

typedef void (*cb_ptr)(double val);


double v_l_actual;
double v_r_actual;

ros_publisher_t pub;

void CCONV positionChangeHandler_left(PhidgetEncoderHandle ch, void *ctx, int positionChange, double timeChange, int indexTriggered);

void CCONV positionChangeHandler_right(PhidgetEncoderHandle ch, void *ctx, int positionChange, double timeChange, int indexTriggered);


bool assert_driving_command();

void set_duty_cycle(PhidgetDCMotorHandle *handle, double duty_cycle);

void calculate_duty_cycle(mc_pid_t *pid, double *duty_cycle, double setpoint, double actual_value, double battery_voltage);

void get_speed(encoder_t *encoder, double *v_actual);

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg);

bool connect_controller(PhidgetDCMotorHandle *ch, controller_side_t side);

bool connect_encoder(PhidgetEncoderHandle *ch, controller_side_t side);

bool connect_position_controller(PhidgetMotorPositionControllerHandle *ch, controller_side_t side);

bool connect_temp_sens(PhidgetTemperatureSensorHandle *ch, controller_side_t side);

bool connect_current_sens(PhidgetCurrentInputHandle *ch, controller_side_t side);

double calculate_r(double v_lin, double v_ang);

void calculate_v(geometry_msgs::Twist cmd_vel, double *v_l, double *v_r);

void report_device_info(PhidgetHandle handle);

void publish_sensor_values(ros_publisher_t *pub, PhidgetCurrentInputHandle *ch_current, PhidgetTemperatureSensorHandle *ch_temp, double v_l_actual, double v_r_actual);

void calculate_distance(double speed, double* distance);




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

  mc_state_t state = 0x00;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

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

  ros::Rate loop_rate(FREQUENCY);

  PhidgetMotorPositionController_setOnPositionChangeHandler(pos_ctrl[right]->handle, positionChangeHandler_right, NULL);
  PhidgetMotorPositionController_setOnPositionChangeHandler(pos_ctrl[left]->handle, positionChangeHandler_left, NULL);


  std_msgs::String msg;

  double v_l_setpoint;
  double v_r_setpoint;
  double dc_l;
  double dc_r;

  v_l_actual = 0;
  v_r_actual = 0;

  mc_pid_t pid[2];

  pid[left].dt = 1 / FREQUENCY;
  pid[left].integral = 0;
  pid[left].last_error = 0;

  pid[right].dt = 1 / FREQUENCY;
  pid[right].integral = 0;
  pid[right].last_error = 0;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    switch(state){
      case init:  // Initialize variables and ros publishers and subscribers; create controller handles
      bool ret;
      controller_t controller[2];

      ros::init(argc, argv, "motor_controller_node");

      ros::NodeHandle n;

      pub.temp_l = n.advertise<sensor_msgs::Temperature>("temp_l", 1000);
      pub.temp_r = n.advertise<sensor_msgs::Temperature>("temp_r", 1000);
      pub.current_l = n.advertise<std_msgs::Float64>("current_l", 1000);
      pub.current_r = n.advertise<std_msgs::Float64>("current_r", 1000);
      pub.speed_l = n.advertise<std_msgs::Float64>("speed_l", 1000);
      pub.speed_r = n.advertise<std_msgs::Float64>("speed_r", 1000);
      ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmd_vel_cb);

      ret = initialize_controller(&controller[left], left);
      if(!ret){
        ROS_INFO("Error initializing the left motor controller; aborting");
        state = deinit;
        break;
      }

      ret = initialize_controller(&controller[right], right);
      if(!ret){
        ROS_INFO("Error initializing the right motor controller; aborting");
        state = deinit;
        break;
      }

      state = connect_controller;
      break;

      case connect_controller:  // connect and configure all the controller devices

        connect_position_controller(&controller[left].ctrl_handle, left);
        connect_position_controller(&controller[left].ctrl_handle, right);


      break;

      case wait_for_rtd:    // check if the robot is OK and wait for the ready to drive command

      break;

      case drive: // drive the robot

      break;

      case brake: //brake to a standstill (preferrably fast)

      break;

      case disengaged:  // disengage the controll loop of the controllers to allow tire movements

      break;

      case deinit:  // close all connections and shutdown

      break;

    }
    if (assert_driving_command())
    {
      calculate_v(cmd_vel, &v_l_setpoint, &v_r_setpoint); // calculate the setpoint speeds for both wheels

      calculate_driving_command();
      calculate_driving_command();
      // get_speed(encoder[left], &v_l_actual); // caculate the actual speed of the wheels
      // get_speed(encoder[right], &v_r_actual);

      // calculate_duty_cycle(&pid[left], &dc_l, v_l_setpoint, v_l_actual, 24);  // calculate the duty cycle using a simple P controller
      // calculate_duty_cycle(&pid[right], &dc_r, v_r_setpoint, v_r_actual, 24); // with a feedforward component

      // set_duty_cycle(mc_handles[left], dc_l); // apply the calculated duty cycle values
      // set_duty_cycle(mc_handles[right], dc_r);

    }
    else{

    }

    publish_sensor_values(&pub, ch_current, ch_tmp, v_l_actual, v_r_actual);


    //PhidgetDCMotor_setTargetVelocity(ch, cmd_vel.linear.x);

    //ROS_INFO("%s", msg.data.c_str());
    // ROS_INFO("Temperatur: %g", temp);
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

bool assert_driving_command()
{
  return true;
}

void set_duty_cycle(PhidgetDCMotorHandle *handle, double duty_cycle)
{
  ROS_INFO("duty cycle: %g", duty_cycle);
  PhidgetReturnCode res;
  res = PhidgetDCMotor_setTargetVelocity(*handle, duty_cycle);
}

void calculate_duty_cycle(mc_pid_t *pid, double *duty_cycle, double setpoint, double actual_value, double battery_voltage)
{
  // Feedforward
  //  Calculating the estimated duty cycle needed for the requested engine speed.
  double dc_ff = setpoint / (battery_voltage * NOMINAL_MOTOR_SPEED * TIRE_CIRCUMFERENCE / NOMINAL_MOTOR_VOLTAGE);

  // Feedback loop
  //  A basic PID controller that corrects the error from the engine load
  double error = setpoint - actual_value;

  double dc_p = K_P * error; // P compontent

  pid->integral = pid->integral + (error * pid->dt);
  double dc_i = K_I * pid->integral; // I component

  if(pid->integral > PID_I_MAX){
    pid->integral = PID_I_MAX;
  }

  if(pid->integral < -PID_I_MAX){
    pid->integral = -PID_I_MAX;
  }

  double dc_d = K_D * (error - pid->last_error) / pid->dt; // D component

  //ROS_INFO("feedforward: %g, dc_P: %g, dc_I: %g, dc_D: %g", dc_ff, dc_p, dc_i, dc_d);
  // adding feedforward and feedback loop
  *duty_cycle = dc_ff + dc_p + dc_i;
  //ROS_INFO("duty_cycle: %g", *duty_cycle);

  pid->last_error = error;

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

  //ROS_INFO("v_l: %g, v_r: %g", *v_l_setpoint, *v_r_setpoint);
  //ROS_INFO(" ");
}

void publish_sensor_values(ros_publisher_t *pub, PhidgetCurrentInputHandle *ch_current, PhidgetTemperatureSensorHandle *ch_temp, double v_l_actual, double v_r_actual)
{
  std_msgs::Float64 current;
  PhidgetCurrentInput_getCurrent(ch_current[left], &current.data);
  pub->current_l.publish(current);

  PhidgetCurrentInput_getCurrent(ch_current[right], &current.data);
  pub->current_r.publish(current);

  sensor_msgs::Temperature temp;

  PhidgetTemperatureSensor_getTemperature(ch_temp[left], &temp.temperature);
  pub->temp_l.publish(temp);

  PhidgetTemperatureSensor_getTemperature(ch_temp[right], &temp.temperature);
  pub->temp_r.publish(temp);


}


class motor_controller{

  public:
    int hub_port;
    int hub_sn;

    PhidgetMotorPositionControllerHandle ctrl_hdl;
    PhidgetCurrentInputHandle current_hdl;
    PhidgetTemperatureSensorHandle temp_hdl;

    motor_controller(int hub_port, int hub_sn){
      this.hub_port = hub_port;
      this.hub_sn = hub_sn;
      this.connected = false;
    }

    double get_speed(){   //returns the wheel speed in m/s
      return speed;
    }

    double get_temperature(){   //returns the temperature in °C
      return temperature;
    }

    double get_current(){   //returns the motor current in A
      return current;
    }

    bool get_connected(){   //checks if the motor controller is connected/if the connection was lost

    }

    void attach_speed_cb(cb_ptr ptr){   //attaches the given callback function to the speed update event (if a speed update event occurs, the given function is called and provoided with the new speed value) 
      speed_cb = ptr;
    }

    void attach_current_cb(cb_ptr ptr){   //attaches the given callback function to the current update event (if a current update event occurs, the given function is called and provoided with the new current value) 
      current_cb = ptr;
    }
    
    void attach_temperature_cb(cb_ptr ptr){   //attaches the given callback function to the temperature update event (if a temperature update event occurs, the given function is called and provoided with the new temperature value) 
      temperature_cb = ptr;
    }

    bool connect(){   //connect to all relevant channels on the controller
      if(!connect_position_controller()){
        return false;
      }
      
      if(!connect_current_sens()){
        return false;
      }
      
      if(!connect_temp_sens()){
        return false;
      }
      connected = true;
      return true;
    }

    bool set_speed(double v){   //set the motor speed to the given value

    }

    bool engage_motors(bool engage){    //engage or disengage the motor position controller

    }

    bool enable_watchdog(int time){   //enables the motor controllers internal watchdog

    }

    bool reset_watchdog(){    //resets the motor controllers internal watchdog. if this is not called within the time specified above, the motors stop and the controller disconnects

    }

    bool set_controller_parameters(double k_p, double k_i, double k_d){   //sets the parameters for the internal PID controller of the motor position controller

    }


  private:
    bool connected;
    double speed;
    double temperature;
    double current;

    cb_ptr speed_cb;
    cb_ptr current_cb;
    cb_ptr temperature_cb;

    void CCONV positionChangeHandler(PhidgetEncoderHandle ch, void *ctx, int positionChange, double timeChange, int indexTriggered)
    {
    speed = (((double)positionChange / timeChange) / (ENCODER_RESOLUTION * GEAR_RATIO)) * TIRE_CIRCUMFERENCE * 1000;
    }

    void CCONV currentChangeHandler(PhidgetCurrentInputHandle ch, void *ctx, double current) {
      this.current = current;
    }

    void CCONV temperatureChangeHandler(PhidgetTemperatureSensorHandle ch, void *ctx, double temperature) {
      this.temperature = temperature;
    }

    bool connect_current_sens()
    {
      PhidgetReturnCode res;
      PhidgetCurrentInput_create(&current_hdl);

      Phidget_setDeviceSerialNumber((PhidgetHandle)current_hdl, hub_sn);
      Phidget_setHubPort((PhidgetHandle)current_hdl, hub_port);

      ROS_INFO("Connecting to current sensor...");

      res = Phidget_openWaitForAttachment((PhidgetHandle)current_hdl, PHIDGET_TIMEOUT_DEFAULT);
      if (res != EPHIDGET_OK)
      {
        ROS_INFO("Connection to the current sensor failed; error 0x%x", res);
        return false;
      }
      else
      {
        ROS_INFO("Connected");
        PhidgetCurrentInput_setDataInterval(*ch, MOTOR_DATA_INTERVALL);
        PhidgetCurrentInput_setOnCurrentChangeHandler(ch, currentChangeHandler, NULL);
        return true;
      }
    }

    bool connect_temp_sens()
    {
      PhidgetReturnCode res;
      PhidgetTemperatureSensor_create(&temp_hdl);

      Phidget_setDeviceSerialNumber((PhidgetHandle)temp_hdl, hub_sn);
      Phidget_setHubPort((PhidgetHandle)temp_hdl, hub_port);

      ROS_INFO("Connecting to temperature sensor...");

      res = Phidget_openWaitForAttachment((PhidgetHandle)temp_hdl, PHIDGET_TIMEOUT_DEFAULT);
      if (res != EPHIDGET_OK)
      {
        ROS_INFO("Connection to the temp sensor failed; error 0x%x", res);
        return false;
      }
      else
      {
        ROS_INFO("Connected");
        PhidgetTemperatureSensor_setDataInterval(temp_hdl, 500);
        PhidgetTemperatureSensor_setOnTemperatureChangeHandler(ch, temperatureChangeHandler, NULL);
        return true;
      }
    }

    bool connect_position_controller()
    {
      PhidgetReturnCode res;
      PhidgetMotorPositionController_create(&ctrl_hdl);

      Phidget_setDeviceSerialNumber((PhidgetHandle)ctrl_hdl, hub_sn);
      Phidget_setHubPort((PhidgetHandle)ctrl_hdl, hub_port);

      ROS_INFO("Connecting to position controller...");

      res = Phidget_openWaitForAttachment((PhidgetHandle)ctrl_hdl, PHIDGET_TIMEOUT_DEFAULT);
      if (res != EPHIDGET_OK)
      {
        ROS_INFO("Connection Failed; code: 0x%x", res); // Exit in error
        return false;
      }
      else
      {
        ROS_INFO("Connected");
        PhidgetMotorPositionController_setDataInterval(ctrl_hdl, ENCODER_DATA_INTERVALL);
        PhidgetMotorPositionController_setAcceleration(ctrl_hdl, POSITION_CONTROLLER_ACCELERATION);
        PhidgetMotorPositionController_setCurrentLimit(ctrl_hdl, MOTOR_CURRENT_LIMIT);
        PhidgetMotorPositionController_setFanMode(ctrl_hdl, FAN_MODE_AUTO);

        PhidgetMotorPositionController_setOnPositionChangeHandler(ctrl_hdl, positionChangeHandler, NULL);

        PhidgetMotorPositionController_setKd(ctrl_hdl, K_D);
        PhidgetMotorPositionController_setKi(ctrl_hdl, K_I);
        PhidgetMotorPositionController_setKp(ctrl_hdl, K_P);
        return true;
      }
    }

    bool check_connection(){

    }


}