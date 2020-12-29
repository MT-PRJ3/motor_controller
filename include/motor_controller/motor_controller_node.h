#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Twist.h"

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
#define MIN_US_RANGE 0.35                   // [m]
#define STANDING_THRESHHOLD 0.1             // [m/s]
#define MAX_CONTROLLER_TEMP 75              // [°C]

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

#define POSITION_CONTROLLER_DATA_INTERVAL 20          // [ms]
#define POSITION_CONTROLLER_FAILSAFE_TIMER 500        // [ms]
#define POSITION_CONTROLLER_CURRENT_REGULATOR_GAIN 10 // [?]
#define POSITION_CONTROLLER_ACCELERATION 50000        // [?]
//#define POSITION_CONTROLLER_RESCALE_FACTOR 5400 // encoder steps per revolution

#define K_P 0.5 // P component of the PID     I dont know what im doing, should probably be changed!
#define K_I 1   // I component of the PID
#define K_D 0   // D component of the PID

typedef void (*cb_ptr)(double val);

struct mc_config_t
{
    int hub_port;
    int hub_sn;
    double tire_circumference;
    double encoder_resolution;
    double gear_ratio;
    double acceleration;
    double current_limit;
    uint32_t watchdog_time;
    bool invert_direction;
    ros::Publisher temp_pub;
    ros::Publisher current_pub;
    ros::Publisher speed_pub;
};

enum robot_error_t
{
    robot_ok = 0x00,
    vBat_low = 0x01,
    bumper_hit = 0x02,
    sc_open = 0x03,
    charger_connected = 0x04,
    us_range_low = 0x05,
    controller_over_temp = 0x06,
    vBat_timeout = 0x11,
    bumper_timeout = 0x12,
    sc_timeout = 0x13,
    charging_timeout = 0x14,
    us_timeout = 0x15,
    cmd_vel_timeout = 0x17
};

enum controller_side_t
{
    left = 0x00,
    right = 0x01
};

class Motor_controller
{

public:
    PhidgetMotorPositionControllerHandle ctrl_hdl;
    PhidgetCurrentInputHandle current_hdl;
    PhidgetTemperatureSensorHandle temp_hdl;
    PhidgetEncoderHandle encoder_hdl;

    Motor_controller(mc_config_t config);

    ~Motor_controller();

    bool connect();

    double get_speed();

    double get_temperature();

    double get_current();

    // void attach_speed_cb(cb_ptr ptr);

    // void attach_current_cb(cb_ptr ptr);

    // void attach_temperature_cb(cb_ptr ptr);

    bool check_connection();

    bool set_speed(double v);

    bool engage_motors(bool engage);

    bool set_controller_parameters(double k_p, double k_i, double k_d);

    void report_device_info();

    void speed_cb(double speed);
    void current_cb(double current);
    void temperature_cb(double temperature);

private:
    bool connected;
    int hub_port;
    int hub_sn;
    double speed;
    double temperature;
    double current;

    cb_ptr ext_speed_cb;
    cb_ptr ext_current_cb;
    cb_ptr ext_temperature_cb;

    ros::Publisher temp_pub;
    ros::Publisher current_pub;
    ros::Publisher speed_pub;

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

robot_error_t sensor_status();

double calculate_r(double v_lin, double v_ang);

void calculate_v(geometry_msgs::Twist cmd_vel, double *v_l, double *v_r);
