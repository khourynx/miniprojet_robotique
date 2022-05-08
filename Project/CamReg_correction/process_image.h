#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
//float get_distance_cm_blue(void);
//float get_distance_cm_green(void);
//uint16_t get_lineWidth_green(void);
uint16_t get_lineWidth(void);
uint32_t get_mean_image(void);
//uint16_t get_lineWidth_blue(void);
uint16_t get_line_position(void);
void process_image_start(void);

#endif /* PROCESS_IMAGE_H */
