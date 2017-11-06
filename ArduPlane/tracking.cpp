#include "Plane.h"

/**
   handle an updated position from the skydiver
   Clara Todd
 */
void Plane::tracking_update_position(const mavlink_global_position_int_t &msg)
{
    skydiver.location.lat = msg.lat;
    skydiver.location.lng = msg.lon;
    skydiver.last_update_us = AP_HAL::micros();
    skydiver.last_update_ms = AP_HAL::millis();
	
	if (msg.lat != 0 || msg.lon!=0){
		skydiver.location_valid = true;
	}
	else {
		skydiver.location_valid = false;
	}
	
	
	calculate_bearing_and_distance();
	// log GPS message
    if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
        Log_Write_Skydiver_GPS();
    }
}

/*
	Clara Todd - This function calculates the skydiver azimuth and range from the pixy camera 
*/
void Plane::get_pixy_block(void){
	
	int negative_X = 0;
	
	int posx = 0;
	int posy = 0;
	
	// get new sensor data
    pixy.update();
	
	//Check we have a valid target from Pixy camera
    if (pixy.num_targets() > 0 && pixy.last_update_ms() != skydiver.last_pixy_meas_time_ms) {
		// Get pixel coordinates of target
		pixy.get_unit_vector_body(skydiver.pixy_pixel_position_x, skydiver.pixy_pixel_position_y, skydiver.pixy_pixel_size_x, skydiver.pixy_pixel_size_y);
		
		//Change to be +/- 160 pixels in x dimension and +/- 100 pixels in y dimension
		posx = skydiver.pixy_pixel_position_x - 160;
		posy = skydiver.pixy_pixel_position_y - 100;
		
		//Store negative value and make values positive as LUT is symmetric so only half the values are stored.
		if (posx <0){
			negative_X = 1;
			posx *= -1;
		}
        if (posy <0){
			posy *= -1;
		}
		if (posx > 158){
			posx = 158;
		}
		posy /= 2; //Divide by two due to compression of LUT
		
		//Undistort pixel positions
		posx = LUTX[posy][posx];
		
		if (negative_X){
			posx *= -1;
		}
		
		//Calculate azimuth
		skydiver.pixy_angle_x = atanf(posx/242.414)*180/M_PI;
		
		//Calculate Range
		skydiver.pixy_range = (0.2*242.414/skydiver.pixy_pixel_size_y + 0.2*163.827/skydiver.pixy_pixel_size_x)/2;
		
		skydiver.azimuth = skydiver.pixy_angle_x;
		UAV_spin = false;
        skydiver.last_pixy_meas_time_ms = pixy.last_update_ms();
    }
	
	// Data Fusion Algorithm
	else if (skydiver.last_pixy_meas_time_ms<AP_HAL::millis()-200){
		if(skydiver.location_valid && skydiver.GPS_distance>3 && skydiver.last_update_ms>AP_HAL::millis()-500){
			skydiver.azimuth = skydiver.GPS_angle;
			UAV_spin = false;
		}
		else{
			UAV_spin = true;
		}
	}
		
	
	// log Pixy message
	if (should_log(MASK_LOG_GPS) && !ahrs.have_ekf_logging()) {
		Log_Write_Skydiver_Pixy();
	}
}


// Use AP_MATH/location functions to return bearing and distance between UAV and Skydiver
// Clara Todd
void Plane::calculate_bearing_and_distance(void)
{
	skydiver.GPS_bearing = get_bearing_cd(current_loc, skydiver.location)/100.0;
	//From test.cpp
	if (compass.read()) {
		
		UAVHeading = (ahrs.yaw_sensor / 100.0);
		
		skydiver.GPS_angle = skydiver.GPS_bearing-UAVHeading-magnetic_declination; // magnetic_decliantion
		if (skydiver.GPS_angle > 180){
			skydiver.GPS_angle-=360.0;
		}
		if (skydiver.GPS_angle < -180){
			skydiver.GPS_angle+=360.0;
		}

    }
	skydiver.GPS_distance = get_distance(current_loc, skydiver.location);
}

