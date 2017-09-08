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



void Plane::get_pixy_block(void){
	
	int negative_X = 0;
	
	int posx = 0;
	int posy = 0;
	
	Vector3f velocity;
	ahrs.get_velocity_NED(velocity);
	//hal.console->printf("x: %f ", velocity.x);
	//hal.console->printf("y: %f \n", velocity.y);
	
	// get new sensor data
    pixy.update();
	
    //hal.console->printf("n_t: %i", pixy.num_targets());
	//hal.console->printf("%i", pixy.last_update_ms());
	//hal.console->printf("%i", skydiver.last_pixy_meas_time_ms);
    if (pixy.num_targets() > 0 && pixy.last_update_ms() != skydiver.last_pixy_meas_time_ms) {
		skydiver.camera_lock = true;
        //pixy.get_angle_to_target_rad(skydiver.pixy_angle_x, skydiver.pixy_angle_y);
		
		pixy.get_unit_vector_body(skydiver.pixy_pixel_position_x, skydiver.pixy_pixel_position_y, skydiver.pixy_pixel_size_x, skydiver.pixy_pixel_size_y);
		//hal.console->printf("position: %u\n", skydiver.pixy_pixel_position_x);
		
		posx = skydiver.pixy_pixel_position_x - 160;
		posy = skydiver.pixy_pixel_position_y - 100;
		
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
		posx = LUTX[posy][posx];
		
		if (negative_X){
			posx *= -1;
		}
		
		skydiver.pixy_angle_x = atanf(posx/242.414)*180/M_PI;
		
		UAV_spin = false;
		
		skydiver.azimuth = skydiver.pixy_angle_x;
		
		hal.console->printf("Azimuth: %f\n", skydiver.pixy_angle_x);
		
        skydiver.last_pixy_meas_time_ms = pixy.last_update_ms();
		// Debug Statement to check angle received
		//hal.console->printf("x: %f\n", skydiver.pixy_angle_x);
		
    }
	else if (skydiver.last_pixy_meas_time_ms<AP_HAL::millis()-200){
		skydiver.camera_lock = false;
		UAV_spin = true;
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
		
		skydiver.GPS_angle = UAVHeading-skydiver.GPS_bearing-magnetic_declination; // magnetic_decliantion
		if (skydiver.GPS_angle > 180){
			skydiver.GPS_angle-=360.0;
		}
		if (skydiver.GPS_angle < -180){
			skydiver.GPS_angle+=360.0;
		}

    }
	skydiver.GPS_distance = get_distance(current_loc, skydiver.location);
}

