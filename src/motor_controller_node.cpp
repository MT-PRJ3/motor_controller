#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
//#include "phidgets_api/motor.h"
//#include "phidgets_api/phidget.h"
#include <stdlib.h>
#include "phidget22.h"

#include <string.h>


#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

geometry_msgs::Twist cmd_vel;

void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel = *msg;
}

bool connect_controller(PhidgetDCMotorHandle *ch){

  PhidgetReturnCode res;

	PhidgetDCMotor_create(ch);

  ROS_INFO("Connecting to device...");

	res = Phidget_openWaitForAttachment((PhidgetHandle)*ch, 1000);
	if (res != EPHIDGET_OK){
		ROS_INFO("Connection Failed; code: 0x%x", res); // Exit in error
    return false;
  }
  else{
    ROS_INFO("Connectd");
    return true;
  }
}

bool connect_temp_sens(PhidgetTemperatureSensorHandle* ch){
	
  PhidgetReturnCode res;
  PhidgetTemperatureSensor_create(ch);

  ROS_INFO("Connecting to the temperature sensor");

  res = Phidget_openWaitForAttachment((PhidgetHandle)*ch, PHIDGET_TIMEOUT_DEFAULT);
	if (res != EPHIDGET_OK){
    ROS_INFO("Connection to the temp sensor failed; error 0x%x", res);
		return false;
  }
  else{
    ROS_INFO("Connection to the temp sensor successful");
    return true;
  }


}

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

  ros::Rate loop_rate(10);

	PhidgetDCMotorHandle ch;

  if(connect_controller(&ch)){
    PhidgetDCMotor_setFanMode(ch, FAN_MODE_ON);
    // PhidgetDCMotor_setTargetVelocity(ch, 0.35);    
  }

  PhidgetTemperatureSensorHandle ch_tmp;

  if(connect_temp_sens(&ch_tmp)){
    double temp;
    PhidgetTemperatureSensor_getTemperature(ch_tmp, &temp);
    ROS_INFO("Temperatur: %g", temp);
  }
  


  

  // phidgets::MotorController* controller = new phidgets::MotorController();
  // ROS_INFO("Trying to connect to any MotorController");    

  // ROS_INFO("Supply Voltage: %g", controller->getSupplyVoltage());
  std_msgs::String msg;
  // std::stringstream ret;

  // int err_code = phidget->openAndWaitForAttachment(620709, 1000);
  // if(err_code == 0){
  //   ROS_INFO("Connected successfully!");
  // }
  // else{
  //   ret << phidget->getErrorDescription(err_code);
  //   msg.data = ret.str();
  //   ROS_INFO("%s", msg.data.c_str());
  //   ROS_INFO("(ERROR_CODE %d)", err_code);
  // }
  

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    std::stringstream ss;
    ss << std::to_string(cmd_vel.linear.x) << " " << count;
    msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

	PhidgetDCMotor_delete(&ch);



  return 0;
}