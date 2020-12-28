#include "motor_controller/motor_controller_node.h"

enum mc_state_t
{
  init = 0x00,
  connect_controllers,
  wait_for_rtd,
  drive,
  brake,
  disengaged,
  deinit
};

typedef struct
{
  ros::Publisher temp_l;
  ros::Publisher temp_r;
  ros::Publisher current_l;
  ros::Publisher current_r;
  ros::Publisher speed_l;
  ros::Publisher speed_r;
} ros_publisher_t;

typedef struct bool_stamped_t
{
  std_msgs::Bool msg;
  ros::Time time;
};

typedef struct float_stamped_t
{
  std_msgs::Float32 msg;
  ros::Time time;
};





ros_publisher_t pub;

geometry_msgs::TwistStamped cmd_vel;
bool_stamped_t bumper;
bool_stamped_t saftey_circuit;
bool_stamped_t charging;
float_stamped_t bat_voltage;
sensor_msgs::Range us[6];


mc_state_t state = init;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller_node");

  ros::NodeHandle n;

  ros::Rate loop_rate(FREQUENCY);

  pub.temp_l = n.advertise<sensor_msgs::Temperature>("temp_l", 1000);
  pub.temp_r = n.advertise<sensor_msgs::Temperature>("temp_r", 1000);
  pub.current_l = n.advertise<std_msgs::Float64>("current_l", 1000);
  pub.current_r = n.advertise<std_msgs::Float64>("current_r", 1000);
  pub.speed_l = n.advertise<std_msgs::Float64>("speed_l", 1000);
  pub.speed_r = n.advertise<std_msgs::Float64>("speed_r", 1000);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 1000, cmd_vel_cb);
  ros::Subscriber rtd_bttn_sub = n.subscribe("rtd", 1000, rtd_bttn_cb);
  ros::Subscriber bumper_sub = n.subscribe("bumper", 1000, bumper_cb);
  ros::Subscriber sc_sub = n.subscribe("safety_circuit", 1000, sc_cb);
  ros::Subscriber charging_sub = n.subscribe("charging", 1000, charging_cb);
  ros::Subscriber bat_voltage_sub = n.subscribe("vBat", 1000, vbat_cb);
  ros::Subscriber ultrasound_0_sub = n.subscribe("ultrasound_0", 1000, us_0_cb);
  ros::Subscriber ultrasound_1_sub = n.subscribe("ultrasound_1", 1000, us_1_cb);
  ros::Subscriber ultrasound_2_sub = n.subscribe("ultrasound_2", 1000, us_2_cb);
  ros::Subscriber ultrasound_3_sub = n.subscribe("ultrasound_3", 1000, us_3_cb);
  ros::Subscriber ultrasound_4_sub = n.subscribe("ultrasound_4", 1000, us_4_cb);
  ros::Subscriber ultrasound_5_sub = n.subscribe("ultrasound_5", 1000, us_5_cb);

  state = connect_controllers;

  mc_config_t config = {
      .hub_port = left,
      .hub_sn = VINT_SN,
      .tire_circumference = TIRE_CIRCUMFERENCE,
      .encoder_resolution = ENCODER_RESOLUTION,
      .gear_ratio = GEAR_RATIO,
      .acceleration = POSITION_CONTROLLER_ACCELERATION,
      .current_limit = MOTOR_CURRENT_LIMIT,
      .watchdog_time = 500,
      .invert_direction = false};

  Motor_controller *controller[2];

  controller[left] = new Motor_controller(config);

  config.invert_direction = true;
  config.hub_port = right;

  controller[right] = new Motor_controller(config);

  double v_l;
  double v_r;

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    switch (state)
    {

    case connect_controllers: // connect and configure all the controller devices
      if (!controller[left]->connect())
      {
        ROS_ERROR("connection to the left controller failed");
      }
      else
      {
        controller[left]->report_device_info();
        if (!controller[right]->connect())
        {
          ROS_ERROR("connection to the right controller failed");
        }
        else
        {
          controller[right]->report_device_info();
          state = wait_for_rtd;
        }
      }
      break;

    case wait_for_rtd: // check if the robot is OK and wait for the ready to drive command
      if (controller[left]->check_connection() && controller[right]->check_connection())
      {
        if (true)
        {
          state = drive;
          controller[left]->engage_motors(true);
          controller[right]->engage_motors(true);
        }
      }
      else
      {
        ROS_ERROR("CONNECTION LOST!");
        state = connect_controllers;
      }

      break;

    case drive: // drive the robot
      if (controller[left]->check_connection() && controller[right]->check_connection())
      {
        if (robot_ok())
        {
          calculate_v(cmd_vel.twist, &v_l, &v_r);
          controller[left]->set_speed(v_l);
          controller[right]->set_speed(v_r);
          ROS_INFO("speeds:");
          ROS_INFO("Speed left: %g", controller[left]->get_speed());
          ROS_INFO("Speed right: %g", controller[right]->get_speed());

        }
        else
        {
          controller[left]->set_speed(0);
          controller[right]->set_speed(0);
          state = brake;
          ROS_WARN("Robot not ok; trying to brake");
        }
      }
      else
      {
        ROS_ERROR("CONNECTION LOST!");
        state = connect_controllers;
      }

      break;

    case brake: //brake to a standstill (preferrably fast)
      if (controller[left]->check_connection() && controller[right]->check_connection())
      {
        if ((controller[left]->get_speed() <= 0.1) && (controller[right]->get_speed() <= 0.1))
        {
          controller[left]->engage_motors(false);
          controller[right]->engage_motors(false);
          state = disengaged;
          ROS_INFO("Robot standing; disengaging motors");
        }
      }
      else
      {
        ROS_ERROR("CONNECTION LOST!");
        state = connect_controllers;
      }
      break;

    case disengaged: // disengage the controll loop of the controllers to allow tire movements
      if (controller[left]->check_connection() && controller[right]->check_connection())
      {
        if(robot_ok()){
          state = wait_for_rtd;
          ROS_INFO("Failstate resolved; waiting for rtd");
        }
      }
      else
      {
        ROS_ERROR("CONNECTION LOST!");
        state = connect_controllers;
      }      
      break;

    case deinit: // close all connections and shutdown

      delete controller[left];
      delete controller[right];
      return -1;

    default:

      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

bool robot_ok(){
  ros::Duration age;

  age = ros::Time::now() - cmd_vel.header.stamp;
  if(age.toSec() > CMD_VEL_TIMEOUT){
    return false;
  }

  age = ros::Time::now() - bumper.time;
  if(age.toSec() > CMD_VEL_TIMEOUT){
    return false;
  }
}

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_vel.twist = *msg;
  cmd_vel.header.stamp = ros::Time::now();
}

void rtd_bttn_cb(const std_msgs::Bool::ConstPtr &msg){
  static bool old;
  if((msg->data) && (!old)){
    //rising flank; button pressed
    ROS_INFO("RTD button pressed");
    if(state == wait_for_rtd){
      state = drive;
    }else
    {
      if(state == drive){
        state = brake;
      }
    }
  }
  old = msg->data;
}

void bumper_cb(const std_msgs::Bool::ConstPtr &msg){
  bumper.msg = *msg;
  bumper.time = ros::Time::now();
}

void sc_cb(const std_msgs::Bool::ConstPtr &msg){
  saftey_circuit.msg = *msg;
  saftey_circuit.time = ros::Time::now();
}

void charging_cb(const std_msgs::Bool::ConstPtr &msg){
  charging.msg = *msg;
  charging.time = ros::Time::now();
}

void vbat_cb(const std_msgs::Float32::ConstPtr &msg){
  bat_voltage.msg = *msg;
  bat_voltage.time = ros::Time::now();

}

void us_0_cb(const sensor_msgs::Range::ConstPtr &msg){
  us[0] = *msg;  
}
void us_1_cb(const sensor_msgs::Range::ConstPtr &msg){
  us[1] = *msg;  
}
void us_2_cb(const sensor_msgs::Range::ConstPtr &msg){
  us[2] = *msg;  
}
void us_3_cb(const sensor_msgs::Range::ConstPtr &msg){
  us[3] = *msg;  
}
void us_4_cb(const sensor_msgs::Range::ConstPtr &msg){
  us[4] = *msg;  
}
void us_5_cb(const sensor_msgs::Range::ConstPtr &msg){
  us[5] = *msg;  
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
    v_lin = cmd_vel.linear.x;
  }

  if (cmd_vel.angular.z != 0)
  {
    state = state + 0x02;
    v_ang = cmd_vel.angular.z;
  }

  switch (state)
  {

  case 0: // v_lin and v_ang = 0 -> dont move
    v_l = 0;
    v_r = 0;
    break;

  case 1: // v_ang = 0 -> only linear motion
  
    if (v_lin > V_LIN_MAX)
    { // Limit maximum linear speed
      if (v_lin > 0)
      {
        v_lin = V_LIN_MAX;
      }
      else
      {
        v_lin = -V_LIN_MAX;
      }
      ROS_INFO("Linear speed exceeds limits -> Reduced speed from %g to %g!", cmd_vel.linear.x, v_lin);
    }

    v_l = v_lin;
    v_r = v_lin;
    break;

  case 2: // v_lin = 0 -> only angular motion
  {
    if (v_ang > V_ANG_MAX)
    { // Limit maximum angular speed
      if (v_ang > 0)
      {
        v_ang = V_ANG_MAX;
      }
      else
      {
        v_ang = -V_ANG_MAX;
      }
      ROS_INFO("Angular speed exceeds limits -> Reduced speed from %g to %g!", cmd_vel.angular.z, v_ang);
    }
    double d_v_ang = v_ang * AXLE_WIDTH; // delta v in [m/s]

    v_l = d_v_ang / 2;
    v_r = -d_v_ang / 2;
  }
  break;

  case 3: // v_lin and v_ang != 0 -> make sure the robot doesnt tip over
  {
    double r = calculate_r(v_lin, v_ang);

    if (v_lin > V_LIN_MAX)
    { // Limit maximum linear speed
      if (v_lin > 0)
      {
        v_lin = V_LIN_MAX;
      }
      else
      {
        v_lin = -V_LIN_MAX;
      }
      ROS_INFO("Linear speed exceeds limits -> Reduced speed from %g to %g!", cmd_vel.linear.x, v_lin);
    }

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

//--Implementation of the Motor_controller class functions-----------------------------------------------------------------------------------
Motor_controller::Motor_controller(mc_config_t config)
{
  this->hub_port = config.hub_port;
  this->hub_sn = config.hub_sn;
  this->inverted = config.invert_direction;

  //Check if the given config values are valid to avoid dividing by 0 etc...
  if (config.tire_circumference <= 0)
  {
    ROS_ERROR("Tire circumference must be greater than 0!");
  }
  else
  {
    this->tire_circumference = config.tire_circumference;
  }

  if (config.encoder_resolution <= 0)
  {
    ROS_ERROR("encoder resolution must be greater than 0!");
  }
  else
  {
    this->encoder_resolution = config.encoder_resolution;
  }

  if (config.gear_ratio <= 0)
  {
    ROS_ERROR("gear ratio must be greater than 0!");
  }
  else
  {
    this->gear_ratio = config.gear_ratio;
  }

  if ((config.acceleration <= 0.1) && (config.acceleration >= 10000000))
  {
    ROS_ERROR("Invalid acceleration value");
  }
  else
  {
    this->acceleration = config.acceleration;
  }

  if ((config.current_limit <= 2.0) && (config.current_limit >= 25.0))
  {
    ROS_ERROR("Invalid current limit");
  }
  else
  {
    this->current_limit = config.current_limit;
  }

  if ((config.watchdog_time <= 500) && (config.watchdog_time >= 30000))
  {
    ROS_ERROR("Invalid current limit");
  }
  else
  {
    this->watchdog_timer = config.watchdog_time;
  }

  this->connected = false;

  PhidgetCurrentInput_create(&current_hdl);
  PhidgetTemperatureSensor_create(&temp_hdl);
  PhidgetMotorPositionController_create(&ctrl_hdl);
  PhidgetEncoder_create(&encoder_hdl);
}

Motor_controller::~Motor_controller()
{
  ROS_INFO("Program stopped; deleting handles...");
  PhidgetMotorPositionController_delete(&ctrl_hdl);
  PhidgetEncoder_delete(&encoder_hdl);
  PhidgetTemperatureSensor_delete(&temp_hdl);
  PhidgetCurrentInput_delete(&current_hdl);
}

double Motor_controller::get_speed()
{ //returns the wheel speed in m/s
  return speed;
}

double Motor_controller::get_temperature()
{ //returns the temperature in °C
  return temperature;
}

double Motor_controller::get_current()
{ //returns the motor current in A
  return current;
}

bool Motor_controller::check_connection()
{ //checks if the motor controller is connected/if the connection was lost and resets the failsafe timer if not
  int attached = 0;
  PhidgetReturnCode ret;
  ret = Phidget_getAttached((PhidgetHandle)ctrl_hdl, &attached);

  if ((attached != 0) && (ret == EPHIDGET_OK))
  { // connection ok -> reset failsafe timer
    // PhidgetMotorPositionController_resetFailsafe(ctrl_hdl);
    connected = true;
    return true;
  }
  else // connection not ok -> return error
  {
    connected = false;
    return false;
  }
}

void Motor_controller::report_device_info()
{
  const char *name;
  int32_t deviceID;
  int32_t SN;
  int32_t port;

  Phidget_getDeviceName((PhidgetHandle)ctrl_hdl, &name);
  Phidget_getDeviceSerialNumber((PhidgetHandle)ctrl_hdl, &SN);
  Phidget_getHubPort((PhidgetHandle)ctrl_hdl, &port);

  ROS_INFO("device %s connected to port %d (VINT SN: %d) ", name, port, SN);
}

bool Motor_controller::connect()
{ //connect to all relevant channels on the controller
  if (!connect_position_controller())
  {
    return false;
  }

  if (!connect_current_sens())
  {
    return false;
  }

  if (!connect_temp_sens())
  {
    return false;
  }

  if (!connect_encoder())
  {
    return false;
  }

  connected = true;
  return true;
}

bool Motor_controller::set_speed(double v)
{ //set the motor speed to the given value
  double pos = 0;
  double speed = 0;
  PhidgetReturnCode ret;

  if (inverted)
  {
    v = -v;
  }

  // set the direction
  if (v > 0)
  {
    pos = max_pos;
  }
  else
  {
    pos = min_pos;
  }
  PhidgetMotorPositionController_setTargetPosition(ctrl_hdl, pos);

  //convert speed from m/s to encoder steps/s
  speed = abs((v / tire_circumference) * encoder_resolution * gear_ratio);

  //set the speed
  ret = PhidgetMotorPositionController_setVelocityLimit(ctrl_hdl, speed);
  if (ret == EPHIDGET_OK)
  {
    return true;
  }
  else
  {
    ROS_ERROR("set speed for hub port %d failed! error code: 0x%x", hub_port, ret);
    return false;
  }
}

bool Motor_controller::engage_motors(bool engage)
{ //engage or disengage the motor position controller
  PhidgetReturnCode ret;

  ret = PhidgetMotorPositionController_setEngaged(ctrl_hdl, (int)engage);
  if (ret == EPHIDGET_OK)
  {
    return true;
  }
  else
  {
    ROS_INFO("setEngaged on port %d returned 0x%x", hub_port, ret);
    return false;
  }
}

bool Motor_controller::set_controller_parameters(double k_p, double k_i, double k_d)
{ //sets the parameters for the internal PID controller of the motor position controller
  if (connected)
  {
    PhidgetMotorPositionController_setKd(ctrl_hdl, k_p);
    PhidgetMotorPositionController_setKi(ctrl_hdl, k_i);
    PhidgetMotorPositionController_setKp(ctrl_hdl, k_d);
    return true;
  }
  else
  {
    return false;
  }
}

void CCONV Motor_controller::positionChangeHandler(PhidgetEncoderHandle ch, void *ctx, int positionChange, double timeChange, int indexTriggered)
{
  *((double *)ctx) = (((double)positionChange / timeChange) / (ENCODER_RESOLUTION * GEAR_RATIO)) * TIRE_CIRCUMFERENCE * 1000;
}

void CCONV Motor_controller::currentChangeHandler(PhidgetCurrentInputHandle ch, void *ctx, double current)
{
  *((double *)ctx) = current;
}

void CCONV Motor_controller::temperatureChangeHandler(PhidgetTemperatureSensorHandle ch, void *ctx, double temperature)
{
  *((double *)ctx) = temperature;
}

bool Motor_controller::connect_current_sens()
{
  PhidgetReturnCode res;

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
    PhidgetCurrentInput_setDataInterval(current_hdl, MOTOR_DATA_INTERVALL);
    PhidgetCurrentInput_setOnCurrentChangeHandler(current_hdl, currentChangeHandler, &current);
    return true;
  }
}

bool Motor_controller::connect_temp_sens()
{
  PhidgetReturnCode res;

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
    PhidgetTemperatureSensor_setOnTemperatureChangeHandler(temp_hdl, temperatureChangeHandler, &temperature);
    return true;
  }
}

bool Motor_controller::connect_position_controller()
{
  PhidgetReturnCode res;

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
    // PhidgetMotorPositionController_enableFailsafe(ctrl_hdl, watchdog_timer);
    PhidgetMotorPositionController_setDataInterval(ctrl_hdl, 20); // 20 ms is the lowes data interval
    PhidgetMotorPositionController_setAcceleration(ctrl_hdl, acceleration);
    PhidgetMotorPositionController_setCurrentLimit(ctrl_hdl, current_limit);
    PhidgetMotorPositionController_setFanMode(ctrl_hdl, FAN_MODE_AUTO);

    PhidgetMotorPositionController_getMinPosition(ctrl_hdl, &min_pos);
    PhidgetMotorPositionController_getMaxPosition(ctrl_hdl, &max_pos);

    // PhidgetMotorPositionController_setKd(ctrl_hdl, K_D); //can be set via the specified function
    // PhidgetMotorPositionController_setKi(ctrl_hdl, K_I);
    // PhidgetMotorPositionController_setKp(ctrl_hdl, K_P);
    return true;
  }
}

bool Motor_controller::connect_encoder()
{
  PhidgetReturnCode res;

  Phidget_setDeviceSerialNumber((PhidgetHandle)encoder_hdl, hub_sn);
  Phidget_setHubPort((PhidgetHandle)encoder_hdl, hub_port);
  ROS_INFO("Connecting to encoder...");
  res = Phidget_openWaitForAttachment((PhidgetHandle)encoder_hdl, PHIDGET_TIMEOUT_DEFAULT);
  if (res != EPHIDGET_OK)
  {
    ROS_INFO("Connection Failed; code: 0x%x", res); // Exit in error
    return false;
  }
  else
  {
    ROS_INFO("Connected");
    PhidgetEncoder_setDataInterval(encoder_hdl, ENCODER_DATA_INTERVALL);
    PhidgetEncoder_setOnPositionChangeHandler(encoder_hdl, positionChangeHandler, &speed);
    return true;
  }
}
