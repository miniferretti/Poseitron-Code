#include <math.h>
#include "CtrlStruct.h"

void beacon_angle(CtrlStruct *theCtrlStruct)
{
	// tower parameters
	double positions_per_rev = theCtrlStruct->theUserStruct->positions_per_rev;

	// get laser informations parameters
	double rising_edge = theCtrlStruct->theCtrlIn->rising_index;
	double falling_edge = theCtrlStruct->theCtrlIn->falling_index;

	// measure angle
	double edge_distance = falling_edge - rising_edge;
	double beacon_angle = (edge_distance / positions_per_rev) * 2 * PI;

	theCtrlStruct->theUserStruct->beacon_angle = beacon_angle;
}

void beacon_distance_tower(CtrlStruct *theCtrlStruct)
{
	// beacon parameters
	double diameter = theCtrlStruct->theUserStruct->diameter;
	double radius = diameter / 2;
	double beacon_angle = theCtrlStruct->theUserStruct->beacon_angle;

	// computing distance from the center of the beacon as the hypothenuse
	// of the rectangle triangle formed by rising or falling edge of the laser
	// and the radius at tangency point, dist_from_beacon is the distance to the
	double beacon_distance_tower = radius / (sin(beacon_angle / 2));

	theCtrlStruct->theUserStruct->beacon_distance_tower = beacon_distance_tower;
}

void beacon_direction(CtrlStruct *theCtrlStruct)
{
	// getting parameters
	double beacon_angle = theCtrlStruct->theUserStruct->beacon_angle;
	double beacon_distance_tower = theCtrlStruct->theUserStruct->beacon_distance_tower;
	double rising_edge = theCtrlStruct->theCtrlIn->rising_index;
	double positions_per_rev = theCtrlStruct->theUserStruct->positions_per_rev;
	double bias_angle = theCtrlStruct->theUserStruct->bias_angle;
	double bias_distance = theCtrlStruct->theUserStruct->bias_angle;

	// angle between reset and rising edge
	double rising_angle = (rising_edge / positions_per_rev) * 2 * PI;

	// direction of the beacon from the axis perpendicular to wheel axis at tower
	double beacon_direction_tower = beacon_angle / 2 + rising_angle;

	// direction of the beacon from the medium point on wheel axis
	double a = beacon_distance_tower * cos(beacon_direction_tower);
	double b = beacon_distance_tower * sin(beacon_direction_tower);
	double beacon_direction = atan(b / (a + bias_distance));

	theCtrlStruct->theUserStruct->beacon_direction_tower = beacon_direction_tower;
	theCtrlStruct->theUserStruct->beacon_direction = beacon_direction;
}

void beacon_distance(CtrlStruct *theCtrlStruct)
{
	// getting parameters
	double beacon_distance_tower = theCtrlStruct->theUserStruct->beacon_distance_tower;
	double beacon_direction_tower = theCtrlStruct->theUserStruct->beacon_direction_tower;
	double beacon_direction = theCtrlStruct->theUserStruct->beacon_direction;
	double radius = 1 / 2 * theCtrlStruct->theUserStruct->diameter;

	// distance from the medium point on wheel axis to center of beacon and then to edge of beacon
	double distance = beacon_distance_tower * sin(beacon_direction_tower) / sin(beacon_direction);
	double beacon_distance = distance - radius;

	theCtrlStruct->theUserStruct->beacon_distance = beacon_distance;
}
