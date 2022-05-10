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


static BSEMAPHORE_DECL(prox_ready_sem, TRUE);

#define ANGLE2STEPS_CONST					PI*STEP_CORRECTION_FACTOR*EPUCK_DIAMETER/(4*WHEEL_PERIMETER)
#define	STEP_CORRECTION_FACTOR	90

static int distance_IR1 = 0;
static int distance_IR8 = 0;

void turn_robot(double angle){
	unsigned int step_goal;
	step_goal = abs(ANGLE2STEPS_CONST*angle/45);
	if(angle > 0)
	{
		right_motor_set_pos(0);
		left_motor_set_pos(step_goal);
		right_motor_set_speed(MOTOR_R);
		left_motor_set_speed(-MOTOR_L);

		while(right_motor_get_pos()<=step_goal && left_motor_get_pos() >=0) {
			__asm__ volatile("nop");
		}
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}
	if(angle < 0)
	{
		right_motor_set_pos(step_goal);
		left_motor_set_pos(0);
		right_motor_set_speed(-MOTOR_R);
		left_motor_set_speed(MOTOR_L);
		while(left_motor_get_pos()<=step_goal && right_motor_get_pos() >=0) {
			__asm__ volatile("nop");
		}
		right_motor_set_speed(0);
		left_motor_set_speed(0);
	}

}

//-------------------------------------------THREADS -----------------------------------------------

static THD_WORKING_AREA(waProximity_sens, 256);
static THD_FUNCTION(Proximity_sens, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;


    while(1){

    	time = chVTGetSystemTime();

    	distance_IR1 = get_calibrated_prox(IR_FRONT_RIGHT);
    	distance_IR8 = get_calibrated_prox(IR_FRONT_LEFT);

    	chBSemSignal(&prox_ready_sem);

    }
}

static THD_WORKING_AREA(waMotor, 1024);
static THD_FUNCTION(Motor, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();

        chBSemWait(&prox_ready_sem);

        if (get_calibrated_prox(IR_FRONT_RIGHT)>2000 || get_calibrated_prox(IR_FRONT_LEFT)>2000){
        	if (get_red_detected()){
        		//turn_robot(ANGLE_RED);
        		right_motor_set_speed(0);
        		left_motor_set_speed(0);
        	}else if (get_blue_detected()){
        		turn_robot(ANGLE_BLUE);
        	}else if (get_green_detected()){
        		turn_robot(ANGLE_GREEN);
        	}

        }else{
    		right_motor_set_speed(MOTOR_R);
    		left_motor_set_speed(MOTOR_L);
            }
    }
}

int get_distance_IR1(void){
	return distance_IR1;
}

int get_distance_IR8(void){
	return distance_IR8;
}

void motor_start(void){
	chThdCreateStatic(waProximity_sens, sizeof(waProximity_sens), NORMALPRIO, Proximity_sens, NULL);
	chThdCreateStatic(waMotor, sizeof(waMotor), NORMALPRIO, Motor, NULL);
}
