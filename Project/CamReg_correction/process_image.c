#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm_red = 0;
static float distance_cm_blue = 0;
static float distance_cm_green= 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t lineWidth_red = 0;
static uint16_t lineWidth_blue = 0;
static uint16_t lineWidth_green = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image_red[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
			image_blue[i/2] = ((uint8_t)img_buff_ptr[i] & 0x07) | ((uint8_t)img_buff_ptr[i+1] & 0xE0);
			image_green[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;
		}

		//search for a line in the image and gets its width in pixels
		lineWidth_red = extract_line_width(image_red);
		lineWidth_blue = extract_line_width(image_blue);
		lineWidth_green = extract_line_width(image_green);

		//converts the width into a distance between the robot and the camera
		if(lineWidth_red) {
			distance_cm_red = PXTOCM/lineWidth_red;
		}

		if(lineWidth_blue) {
			distance_cm_blue= PXTOCM/lineWidth_blue;
		}

		if(lineWidth_green) {
			distance_cm_green = PXTOCM/lineWidth_green;
		}

		if((send_to_computer)&&(lineWidth_red)){
			//sends to the computer the image
			SendUint8ToComputer(image_red, IMAGE_BUFFER_SIZE);
		}else if ((send_to_computer)&&(lineWidth_blue)){
			SendUint8ToComputer(image_blue, IMAGE_BUFFER_SIZE);
		}else if ((send_to_computer)&&(lineWidth_green)){
			SendUint8ToComputer(image_green, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}

float get_distance_cm_red(void){
	return distance_cm_red;
}

uint16_t get_lineWidth_red(void){
	return lineWidth_red;
}

float get_distance_cm_blue(void){
	return distance_cm_blue;
}

uint16_t get_lineWidth_blue(void){
	return lineWidth_blue;
}

float get_distance_cm_green(void){
	return distance_cm_green;
}

uint16_t get_lineWidth_green(void){
	return lineWidth_green;
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
