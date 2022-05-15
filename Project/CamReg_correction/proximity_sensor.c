/* Header:
 * Author/Editor: Marc El Khoury and Joey Kodeih
 * This file is the one where we process the proximity sensors data and command the robot.
 * We call a function from the process image workflow to get what color we detected.
 *
 *
 */

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>

#include <main.h>
#include <sensors/proximity.h>
#include <proximity_sensor.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <process_image.h>

//semaphore of the proximity sensors
static BSEMAPHORE_DECL(prox_ready_sem, TRUE);

#define ANGLEtoSTEPS					PI*STEP_CORRECTION_FACTOR*EPUCK_DIAMETER/(4*WHEEL_PERIMETER)
#define	STEP_CORRECTION_FACTOR	90

static int distance_IR1 = 0;
static int distance_IR8 = 0;

//Function to turn the robot at the angle that we give to the function
void turn_robot(double angle){
	unsigned int step_goal;
	step_goal = abs(ANGLEtoSTEPS*angle/45);
	if(angle > 0){
		right_motor_set_pos(0);
		left_motor_set_pos(step_goal);
		right_motor_set_speed(MOTOR_R);
		left_motor_set_speed(-MOTOR_L);

		while((right_motor_get_pos()<=step_goal && left_motor_get_pos() >=0)){
			__asm__ volatile("nop");
		}
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
	if(angle < 0){
		right_motor_set_pos(step_goal);
		left_motor_set_pos(0);
		right_motor_set_speed(-MOTOR_R);
		left_motor_set_speed(MOTOR_L);
		while((left_motor_get_pos()<=step_goal) && (right_motor_get_pos() >=0)){
			__asm__ volatile("nop");
		}
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}

}

//-------------------------------------------THREADS -----------------------------------------------

// Thread to control the proximity sensors
static THD_WORKING_AREA(waProximity_sens, 256);
static THD_FUNCTION(Proximity_sens, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;


    while(1){

    	time = chVTGetSystemTime();

    	// We stock the distance of the two front proximity sensors
    	distance_IR1 = get_calibrated_prox(IR_FRONT_RIGHT);
    	distance_IR8 = get_calibrated_prox(IR_FRONT_LEFT);

    	chBSemSignal(&prox_ready_sem);	// Signal that the proximity data is captured to let the motor thread starts

    }
}

//Thread to control the Motors

static THD_WORKING_AREA(waMotor, 1024);
static THD_FUNCTION(Motor, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        chBSemWait(&prox_ready_sem);

        //Check if the robot reached the goal distance with the object
        if (get_calibrated_prox(IR_FRONT_RIGHT)>200 || get_calibrated_prox(IR_FRONT_LEFT)>200){
        	//Check the color of the object then turns in the direction of the color detected
        	//We also do a reset after the detection to put our conditions to default after each execution of the thread
        	if (get_red_detected()){
        		turn_robot(ANGLE_RED);
        		reset_function();
        	}else if (get_blue_detected()){
        		turn_robot(ANGLE_BLUE);
        		reset_function();
        	}else if (get_green_detected()){
        		turn_robot(ANGLE_GREEN);
        		reset_function();
        	}else{
        		//If an object is detected in the dark we stop the motors for the safety of the driverless robot and its passengers :)
        		right_motor_set_speed(0);
        		left_motor_set_speed(0);
        	}

        }else{
        	//If nothing is detected we continue to move forward
    		right_motor_set_speed(MOTOR_R);
    		left_motor_set_speed(MOTOR_L);
            }
    }
}

// Start of the threads we called in the main
void motor_start(void){
	chThdCreateStatic(waProximity_sens, sizeof(waProximity_sens), NORMALPRIO, Proximity_sens, NULL);
	chThdCreateStatic(waMotor, sizeof(waMotor), NORMALPRIO, Motor, NULL);
}
