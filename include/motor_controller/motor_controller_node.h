#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Temperature.h"

#include "phidget22.h"

#include <string.h>
#include <sstream>
#include <stdlib.h>
#include <math.h>

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

#define MOTOR_CURRENT_LIMIT 8     // [A]  [2:25]
#define MOTOR_DATA_INTERVALL 100   //data interval in ms [100:60000]
#define ENCODER_DATA_INTERVALL 100 //data interval in ms [100:60000]

#define POSITION_CONTROLLER_DATA_INTERVAL 100          // [ms]
#define POSITION_CONTROLLER_FAILSAFE_TIMER 500        // [ms]
#define POSITION_CONTROLLER_CURRENT_REGULATOR_GAIN 100 // [?]
#define POSITION_CONTROLLER_ACCELERATION 1000000         // [?]
//#define POSITION_CONTROLLER_RESCALE_FACTOR 5400 // encoder steps per revolution

#define K_P 100000 // P component of the PID     I dont know what im doing, should probably be changed!
#define K_I 0   // I component of the PID
#define K_D 0   // D component of the PID

typedef void (*cb_ptr)(double val);

class Motor_controller
{

public:
    int hub_port;
    int hub_sn;
    double speed;
    double temperature;
    double current;

    PhidgetMotorPositionControllerHandle ctrl_hdl;
    PhidgetCurrentInputHandle current_hdl;
    PhidgetTemperatureSensorHandle temp_hdl;
    PhidgetEncoderHandle encoder_hdl;

    Motor_controller(int hub_port, int hub_sn);

    double get_speed();

    double get_temperature();

    double get_current();

    bool get_connected();

    // void attach_speed_cb(cb_ptr ptr);

    // void attach_current_cb(cb_ptr ptr);

    // void attach_temperature_cb(cb_ptr ptr);

    bool connect();

    bool set_speed(double v);

    bool engage_motors(bool engage);

    bool enable_watchdog(int time);

    bool reset_watchdog();

    bool set_controller_parameters(double k_p, double k_i, double k_d);

private:
    bool connected;

    cb_ptr speed_cb;
    cb_ptr current_cb;
    cb_ptr temperature_cb;

    static void CCONV positionChangeHandler(PhidgetEncoderHandle ch, void *ctx, int positionChange, double timeChange, int indexTriggered);

    static void CCONV currentChangeHandler(PhidgetCurrentInputHandle ch, void *ctx, double current);

    static void CCONV temperatureChangeHandler(PhidgetTemperatureSensorHandle ch, void *ctx, double temperature);

    bool connect_current_sens();

    bool connect_temp_sens();

    bool connect_position_controller();

    bool connect_encoder();

    bool check_connection();
};
