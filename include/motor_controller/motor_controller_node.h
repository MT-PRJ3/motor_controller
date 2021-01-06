#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include "phidget22.h"

#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <math.h>

#define FREQUENCY 100 //Hz  Execution frequency of the programm

#define CMD_VEL_TIMEOUT (3 * 1 / FREQUENCY) // timeout for the subscribed topics [s]
#define SC_TIMEOUT 0.5                      // [s]
#define BUMPER_TIMEOUT 0.5                  // [s]
#define US_TIMEOUT 0.5                      // [s]
#define CHARGING_TIMEOUT 0.5                // [s]
#define VBAT_TIMEOUT 0.5                    // [s]
#define MIN_US_RANGE 0.35                   // [m]
#define STANDING_THRESHHOLD 0.1             // [m/s]
#define MAX_CONTROLLER_TEMP 75              // [°C]
#define VBAT_MIN 21                         // [V]

#define VINT_SN 620709

#define V_LIN_MAX 1.5    // [m/s]   Max allowed linear speed (only effective if the given angular speed is 0)
#define V_ANG_MAX 2.0    // [rad/s] Max allowed angular speed (only effective if the given linear speed is 0)
#define A_Y_MAX 2.5      // [m/s²]  Maximum allowed lateral acceleration

#define AXLE_WIDTH 0.375                              // [m]
#define TIRE_RADIUS 0.08                              // [m]
#define TIRE_CIRCUMFERENCE (TIRE_RADIUS * 2.0 * M_PI) // [m]

#define GEAR_RATIO 18              // 18:1 [INPUT:OUTPUT]
#define ENCODER_RESOLUTION 1200    // [Counts/Revolution] (on the engine side of the gearbox)
// According to the datasheet, the encoder has a resolution of 300 Steps per revolution, but its a
//  quadrature encoder, so its effectively 300*4

#define MOTOR_CURRENT_LIMIT 25     // [A]  [2:25]
#define POSITION_CONTROLLER_ACCELERATION 50000        // [?]

typedef void (*cb_ptr)(double val); // Definition of callback function pointers for the motor controller sensor values; not used atm

struct mc_config_t  //Struct containing the configuration values needed to create a Motor_controller Object
{
    int hub_port;   // The number of the vint hub port the device is connected to
    int hub_sn;     // The Serial number of the vint hub the device is connected to. May be set to PHIDGET_SERIALNUMBER_ANY if only one vint hub is used
    double tire_circumference;  // 
    double encoder_resolution;  //
    double gear_ratio;
    double acceleration;        // The max allowed acceleration of the motors (in encoder steps per second)
    double current_limit;       
    uint32_t watchdog_time;     // The timout of the watchdog timer in the Motor controller
    bool invert_direction;      // set to True if the Motor is mounted backwards (the motor direction is reversed)
    cb_ptr temp_cb_fct;    // the publisher that should be used to publish the temperature as soon as it is availible
    cb_ptr current_cb_fct; // the publisher that should be used to publish the current as soon as it is availible
    cb_ptr speed_cb_fct;   // the publisher that should be used to publish the speed as soon as it is availible
};

enum robot_error_t  // The error values that can be returned by the sensor_status function
{
    robot_ok = 0x00,            //Everything is ok; the robot can drive
    vBat_low = 0x01,
    bumper_hit = 0x02,
    sc_open = 0x03,             // the relais control voltage is low (shutdown button pressed or object in laserscanner error field)
    charger_connected = 0x04,   // the charging plug is connected
    us_range_low = 0x05,
    controller_over_temp = 0x06,
    vBat_timeout = 0x11,    
    bumper_timeout = 0x12,
    sc_timeout = 0x13,
    charging_timeout = 0x14,
    us_timeout = 0x15,
    cmd_vel_timeout = 0x17
};

enum controller_side_t  //makes discerning the controllers easier. WARNING: is used for array indexes as well as the vint hub port indx; changing this may brake the code
{
    left = 0x00,
    right = 0x01
};

class Motor_controller  // The motor controller class that handles all the handles etc...
{

public:
    Motor_controller(mc_config_t config);   //constructor. Initializes all the handles and fills the configuration parameters

    ~Motor_controller();    //destructor. closes all connections and clears the handles

    bool connect();         //connects to the position controller on the given vint hub port and all relevant sensors. returns true if all devices are connected

    double get_speed();     //returns the last reported motor speed

    bool new_speed_val();       // returns true if  a new speed value is availible

    double get_temperature();   //returns the last reported motor controller temperature

    bool new_temp_val();        // returns true if a new temprerature value is availible

    double get_current();       //returns the last reported motor controller current
    
    bool new_current_val();     // returns true if  a new current value is availible

    void attach_speed_cb(cb_ptr ptr);

    void attach_current_cb(cb_ptr ptr);

    void attach_temperature_cb(cb_ptr ptr);

    bool check_connection();    //checks if the controller is still responding. returns false if the connection was lost.

    bool set_speed(double v);   //sets the speed the wheel should be spinning at (in m/s)

    bool engage_motors(bool engage);    //engages the motor controller. if the controller is disengaged, the motor is unpowered and the robot can be pushed 

    bool set_controller_parameters(double k_p, double k_i, double k_d); // sets the controller parameters to these (i dont know how this affects the speed controller tho)

    void report_device_info();  //prints the device name, vint hub port and vint hub sn as a ros info

    //DONT USE THESE; THESE ARE ONLY PUBLIC BECAUSE IM STUPID AND THIS IS THE ONLY WAY TO SOLVE THE STUPID PROBLEM THAT THE CALLBACK FUNCTIONS MUST BE STATIC AND THEREFORE CANT CALL PRIVATE FUNCTIONS OF THE OBJECT THAT IT SHOULD HAVE BELONGED TO IN THE FIRST PLACE
    void speed_cb(double speed);    
    void current_cb(double current);
    void temperature_cb(double temperature);

private:
    PhidgetMotorPositionControllerHandle ctrl_hdl;  
    PhidgetCurrentInputHandle current_hdl;
    PhidgetTemperatureSensorHandle temp_hdl;
    PhidgetEncoderHandle encoder_hdl;

    bool connected;
    int hub_port;
    int hub_sn;
    double speed;
    double temperature;
    double current;
    bool new_speed;
    bool new_temp;
    bool new_current;

    cb_ptr temp_cb_fct;
    cb_ptr current_cb_fct;
    cb_ptr speed_cb_fct;

    double tire_circumference;
    double encoder_resolution;
    double gear_ratio;
    double acceleration;
    double current_limit;
    bool inverted;

    uint32_t watchdog_timer;

    double max_pos; //maximum and minimum positions of the position controller
    double min_pos;

    static void CCONV positionChangeHandler(PhidgetEncoderHandle ch, void *ctx, int positionChange, double timeChange, int indexTriggered);

    static void CCONV currentChangeHandler(PhidgetCurrentInputHandle ch, void *ctx, double current);

    static void CCONV temperatureChangeHandler(PhidgetTemperatureSensorHandle ch, void *ctx, double temperature);

    bool connect_current_sens();

    bool connect_temp_sens();

    bool connect_position_controller();

    bool connect_encoder();
};

robot_error_t sensor_status();  // Checks if all sensor values are okay and if the last message of the topic is too old to be valid. returns robot_ok if everything is ok and it is safe to drive the robot

double calculate_r(double v_lin, double v_ang); // calculates the raduis of the curve that the two given speeds would result in

void calculate_v(geometry_msgs::Twist cmd_vel, double *v_l, double *v_r);   // calculates the speeds for the two wheels from the given cmd_vel message
