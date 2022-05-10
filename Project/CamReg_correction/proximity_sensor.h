#ifndef PROXIMITY_SENSOR_H_
#define PROXIMITY_SENSOR_H_

void turn_robot(double angle);
int get_distance_IR1(void);
int get_distance_IR8(void);
uint8_t get_object_nearby_detection (void);
void object_nearby (void);
void proximity_sensor_start(void);
void motor_start(void);

#define IR_FRONT_RIGHT 		0
#define IR_FRONT_LEFT 		7

#endif /* PROXIMITY_SENSOR_H_ */
