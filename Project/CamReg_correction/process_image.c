/* Header :
 * Author/Editor: Marc El Khoury and Joey Kodeih
 *  This file is where we handle the image capture and processing to get the color of the obstacle
 *  It was taken from the TP4 correction on moodle and modified to fit our needs.
*/

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>
#include <process_image.h>
#include <proximity_sensor.h>

#define RED_GAIN 0x4A
#define BLUE_GAIN 0x45
#define GREEN_GAIN 0x5D

static uint8_t Red_detected = 0;
static uint8_t Blue_detected = 0;
static uint8_t Green_detected = 0;


//semaphore of the image
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


// ------------------------------------------------------THREADS----------------------------------------------------------------------
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	po8030_set_awb(0);
	po8030_set_rgb_gain(RED_GAIN, GREEN_GAIN, BLUE_GAIN);
	po8030_set_exposure(2048,0);
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
	uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};
	uint32_t mean_image_red = 0;
	uint32_t mean_image_blue = 0;
	uint32_t mean_image_green = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts each R-G-B pixels in an array and perform a mean of the value in each array
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			image_red[i/2] = ((uint8_t)img_buff_ptr[i]&0xF8 >> 2);
			//extracts 6bits from the middle of the two bytes
			image_green[i/2] = ((uint8_t)img_buff_ptr[i] & 0x07 << 3) | ((uint8_t)img_buff_ptr[i+1] & 0xE0 >> 5);
			//extracts last 5bits of the second byte
			image_blue[i/2] = (uint8_t)img_buff_ptr[i+1]&0x1F;

			mean_image_red+=image_red[i/2];
			mean_image_blue+=image_blue[i/2];
			mean_image_green+=image_green[i/2];
		}

		mean_image_red/=IMAGE_BUFFER_SIZE;
		mean_image_green/=IMAGE_BUFFER_SIZE;
		mean_image_blue/=IMAGE_BUFFER_SIZE;

		//We compare each value and detect the dominant color
		if((mean_image_red>mean_image_green) && (mean_image_red>mean_image_blue)){
			Red_detected=1;
			Blue_detected=0;
			Green_detected=0;
		}else if ((mean_image_blue>mean_image_green) && (mean_image_red<mean_image_blue)) {
			Red_detected=0;
			Blue_detected=1;
			Green_detected=0;
		}else if ((mean_image_green>mean_image_blue) && (mean_image_green>mean_image_red)) {
			Red_detected=0;
			Blue_detected=0;
			Green_detected=1;
		}else{
			Red_detected=0;
			Blue_detected=0;
			Green_detected=0;
		}

    }
}

// Function to reset detection after a turn
void reset_function(void){
	Red_detected=0;
	Blue_detected=0;
	Green_detected=0;
}

// We declared the variable Red_detected as static in this file so that we can call this function in the motor thread and get the variable
uint8_t get_red_detected(void){
	return Red_detected;
}

// We declared the variable Blue_detected as static in this file so that we can call this function in the motor thread and get the variable
uint8_t get_blue_detected(void){
	return Blue_detected;
}

// We declared the variable Green_detected as static in this file so that we can call this function in the motor thread and get the variable
uint8_t get_green_detected(void){
	return Green_detected;
}


// Start of the threads we called in the main
void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
