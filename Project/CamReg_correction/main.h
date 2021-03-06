#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			3.0f
#define MAX_DISTANCE 			25.0f
#define MOTOR_L					450	// steps/s = 5 cm/s
#define MOTOR_R					450	// steps/s = 5 cm/s
#define DISTANCE_LIMITE_VIRAGE  1
#define TURN_FACTOR				360
#define WHEEL_PERIMETER         13 // [cm]
#define PI                  	3.1415926536f
#define WHEEL_DISTANCE      	5.35f    //cm
#define EPUCK_DIAMETER			54

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
