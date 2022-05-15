/*Header:
 * Author/Editor: Marc El Khoury and Joey Kodeih
 * This is the main file where we did the initiation of each peripheral
 * We called the start of the 4 different threads
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/proximity.h>
#include <chprintf.h>

#include <process_image.h>
#include <proximity_sensor.h>

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

//Declaration of the Robot IPC bus for the proximity sensor to work (An assistant gave us that)
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //messagebus int
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();

	//initiation of the proximity sensors and calibration
	proximity_start();
	calibrate_ir();

	//inits the motors
	motors_init();

	//starts the threads for the motor and proximity sensors
	motor_start();
	//starts the threads for the capture and processing of the image
	process_image_start();


    /* Infinite loop. */
    while (1) {


    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
