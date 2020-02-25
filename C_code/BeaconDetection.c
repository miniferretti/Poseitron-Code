#include <math.h>
#include "CtrlStruct.h"

void beacon_angle()
{
	// tower parameters
	double positions_per_rev = ;

	// get laser informations parameters
	double rising_edge = theCtrlStruct->theCtrlIn->rising_index;
	double falling_edge = theCtrlStruct->theCtrlIn->falling_index;

	// measure angle
	double edge_distance = falling_edge - rising_edge;
	double beacon_angle = (edge_distance / positions_per_rev) * 2 * PI;
}

void beacon_distance()
{
	// beacon parameters
	double diameter = ;
	double radius = diameter / 2;
	double beacon_angle = ;

	// computing distance from the center of the beacon as the hypothenuse
	// of the rectangle triangle formed by rising or falling edge of the laser
	// and the radius at tangency point, dist_from_beacon is the distance to the
	double distance = radius / (sin(beacon_angle / 2));
	double dist_from_beacon = distance - radius;
}

void beacon_direction()
{
	// getting parameters
	double beacon_angle = ;
	double beacon_distance = ;
	double rising_edge = ;
	double positions_per_rev = ;
	double bias_angle = ;
	double bias_distance = ;

	// angle between reset and rising edge
	double rising_angle = (rising_edge / positions_per_rev) * 2 * PI;

	// direction of the beacon from the axis perpendicular to wheel axis at tower
	double beacon_direction_tower = beacon_angle / 2 + rising_angle;

	// direction of the beacon from the medium point on wheel axis
	double a = beacon_distance * cos(beacon_direction_tower);
	double b = beacon_distance * sin(beacon_direction_tower);
	double beacon_direction = atan(b / (a + bias_distance));
}
