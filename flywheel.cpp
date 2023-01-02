#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"


void fwControl() {
    #define RPM_TARGET 100 //Target RPM (would like this to be a parameter which should be easy)

    //Feed Forward
    #define FF_GAIN 0 //Feed forward gain (at 0; needs to be tuned)

    // Constants for PID (all set to 0 becuase need to be tuned)
    #define KP 0.00 // Proportional gain
    #define KI 0.00 // Integral gain
    #define KD 0.00 // Derivative gain

    // Variables for storing the error and integral values
    double error, integral, prev_error;

    while (true) {

        double rpm = fw.get_actual_velocity(); // Measure the current RPM of the flywheel

        //FEED FORWARD
        double ff_control_input = FF_GAIN * RPM_TARGET; // Calculate the feedforward control input based on the target RPM

        //PID
        error = RPM_TARGET - rpm; // Calculate the error between the current RPM and the target RPM
        integral += error; // Update the integral value
        double derivative = error - prev_error; // Calculate the derivative of the error
        prev_error = error;
        double pid_control_input = KP * error + KI * integral + KD * derivative; // Calculate the control input using the PID equation

        double control_input = ff_control_input + pid_control_input;

        // Set the flywheel motor speed based on the control input
        fw.move(control_input);

        // Wait for the next iteration
        pros::delay(20);

    }
}