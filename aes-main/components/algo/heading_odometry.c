#include "heading_odometry.h"

#include "robot_constants.h"

#include <math.h>

// global variables
float heading_odometry_current = 0.0;

static photo_encoder_distance_type distance_old;

void heading_odometry_init()
{
}

/**
 * @todo Check that this is okay
 */
void heading_odometry_calculate()
{
    photo_encoder_distance_type distance = photo_encoder_distance;
    distance.left -= distance_old.left;
    distance_right -= distance_old.right;

    float delta_heading = (distance.left - distance.right) / DISTANCE_BETWEEN_WHEELS;

    heading_odometry_current += delta_heading;
}