set CENTERLINE_EQ_A to 142.13236295.    // latitude coefficient for the linear equation
set CENTERLINE_EQ_B to 1.               // longitude coefficient
set CENTERLINE_EQ_C to 81.62849024.     // constant term
set RUNWAY_WEST_THRESHOLD_LAT to -0.0485997.
set RUNWAY_WEST_THRESHOLD_LNG to -74.724375.
set RUNWAY_EAST_THRESHOLD_LAT to -0.0502118560109606.
set RUNWAY_EAST_THRESHOLD_LNG to -74.4899977802028.
set RUNWAY_ALTITUDE to 69.28267.

lock centerline_angular_deviation to (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
lock centerline_linear_deviation to -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.

lock distance_to_runway to latlng(RUNWAY_WEST_THRESHOLD_LAT, RUNWAY_WEST_THRESHOLD_LNG):distance.
lock altitude_above_runway to ship:altitude - RUNWAY_ALTITUDE.

lock glideslope to arctan(altitude_above_runway / distance_to_runway).
lock localizer to arcsin(centerline_linear_deviation / distance_to_runway).

when True then {
    clearscreen.
    print "Glideslope: " + glideslope.
    print "Localizer: " + localizer.
    print "Centerline deviation: " + centerline_linear_deviation.
    print "Distance to runway: " + distance_to_runway.
    preserve.
}

wait until False.
