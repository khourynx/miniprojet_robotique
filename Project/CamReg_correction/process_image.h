#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

void reset_function(void);
uint8_t get_red_detected(void);
uint8_t get_blue_detected(void);
uint8_t get_green_detected(void);
void process_image_start(void);

#define RED_THRESHOLD		80
#define GREEN_THRESHOLD		50
#define BLUE_THRESHOLD		50

#endif /* PROCESS_IMAGE_H */
