/*
 * IRLock.cpp
 *
 *  Created on: Nov 12, 2014
 *      Author: MLandes
 */

#include "IRLock.h"

// retrieve body frame x and y angles (in radians) to target
// returns true if data is available
bool IRLock::get_angle_to_target_rad(float &x_angle_rad, float &y_angle_rad) const
{
    // return false if we have no target
    if (!_flags.healthy) {
        return false;
    }
	
	int16_t corner1_pix_x = _target_info.pos_x - _target_info.size_x/2;
    int16_t corner1_pix_y = _target_info.pos_y - _target_info.size_y/2;
    int16_t corner2_pix_x = _target_info.pos_x + _target_info.size_x/2;
    int16_t corner2_pix_y = _target_info.pos_y + _target_info.size_y/2;
	
	//hal.console->printf("x: %i", irframe.pixel_x);
	//hal.console->printf("y: %i", irframe.pixel_y);

    float corner1_pos_x, corner1_pos_y, corner2_pos_x, corner2_pos_y;
    pixel_to_1M_plane(corner1_pix_x, corner1_pix_y, corner1_pos_x, corner1_pos_y);
    pixel_to_1M_plane(corner2_pix_x, corner2_pix_y, corner2_pos_x, corner2_pos_y);

    // use data from first (largest) object
    x_angle_rad = atanf(0.5f*(corner1_pos_x+corner2_pos_x));
    y_angle_rad = atanf(0.5f*(corner1_pos_y+corner2_pos_y));
    return true;
}

// retrieve body frame unit vector in direction of target
// returns true if data is available
bool IRLock::get_unit_vector_body(Vector3f& ret) const
{
    // return false if we have no target
    if (!_flags.healthy) {
        return false;
    }

    // use data from first (largest) object
    ret.x = _target_info.pos_x;
    ret.y = _target_info.pos_y;
    ret.z = 1.0f;
    ret /= ret.length();
    return true;
}

//Clara Todd added to store pixel positions 
bool IRLock::get_unit_vector_body(uint16_t& retx, uint16_t& rety, uint16_t& sizex, uint16_t& sizey) const
{
    // return false if we have no target
    if (!_flags.healthy) {
        return false;
    }

    // use data from first (largest) object

    retx = _target_info.pos_x;
	rety = _target_info.pos_y;
	sizex = _target_info.size_x;
	sizey = _target_info.size_x;

    return true;
}

void IRLock::pixel_to_1M_plane(float pix_x, float pix_y, float &ret_x, float &ret_y) const
{
    ret_x = (-0.00293875727162397f*pix_x + 0.470201163459835f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                                4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
    ret_y = (-0.003056843086277f*pix_y + 0.3056843086277f)/(4.43013552642296e-6f*((pix_x - 160.0f)*(pix_x - 160.0f)) +
                                                            4.79331390531725e-6f*((pix_y - 100.0f)*(pix_y - 100.0f)) - 1.0f);
}

