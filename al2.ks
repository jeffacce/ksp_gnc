parameter glideslope_target.
parameter speed_target.
run vordme.

// PID flow:
// raw pitch control <-- pitch target control <-- altitude error = 0
// raw roll control <-- roll target control <-- vordme aimpoint heading / heading target control <-- localizer error = 0
// raw yaw control <-- sideslip = 0 / heading target control <-- localizer error = 0

// 5.5 degrees to clear the west side mountains.
// TODO: waypoint/airspeed mode, like ATC vectoring
// TODO: VOR/DME waypoint follow
// TODO: in approach, incorporate a ground clearance term (approaches -infinity as we get closer to the ground, as a penalty)
// BUGGY: flare not robust against high entry v/s. incorporate pre-flare?
// BUGGY: rollout wheel steer.

set CENTERLINE_EQ_A to 142.13236295.    // latitude coefficient for the linear equation
set CENTERLINE_EQ_B to 1.               // longitude coefficient
set CENTERLINE_EQ_C to 81.62849024.     // constant term
set RUNWAY_EASTWARD_HEADING to 90.55.
set RUNWAY_WESTWARD_HEADING to 270.55.
set RUNWAY_WEST_THRESHOLD_LAT to -0.0485997.
set RUNWAY_WEST_THRESHOLD_LNG to -74.724375.
set RUNWAY_WEST_THRESHOLD_LATLNG to latlng(RUNWAY_WEST_THRESHOLD_LAT, RUNWAY_WEST_THRESHOLD_LNG).
set RUNWAY_WEST_THRESHOLD_500_LATLNG to latlng_away_from_source(RUNWAY_WESTWARD_HEADING, 500, RUNWAY_WEST_THRESHOLD_LATLNG).
set RUNWAY_WEST_TOUCHDOWN_ZONE_LAT to -0.04874048.
set RUNWAY_WEST_TOUCHDOWN_ZONE_LNG to -74.6992639.
set RUNWAY_EAST_THRESHOLD_LAT to -0.0502118560109606.
set RUNWAY_EAST_THRESHOLD_LNG to -74.4899977802028.
set RUNWAY_EAST_THRESHOLD_LATLNG to latlng(RUNWAY_EAST_THRESHOLD_LAT, RUNWAY_EAST_THRESHOLD_LNG).
set RUNWAY_ALTITUDE to 70.28267.
set TARGET_ALTITUDE_INTERCEPT to 5.
set ROLL_TARGET_ITERM_HEADING_ERROR_THRESH to 5.
set FLARE_ALTITUDE_THRESH to 20.        // altitude to start flaring.
set FLARE_BEZIER_X_POWER to 2.5.        // positive number. smaller for overflare, larger for underflare.
set TOUCHDOWN_TARGET_VS to -1.          // desired touchdown vertical speed: m/s
set TOUCHDOWN_STATE_ALTITUDE_EPSILON to 0.2.
set HEADING_TARGET_ITERM_CENTERLINE_LINEAR_DEVIATION_EPSILON to 0.5.
set WHEEL_STOP_SURFACE_SPEED_EPSILON to 0.1.
set GEAR_DOWN_LEAD_TIME to 20.          // seconds, before expected touchdown
set DEROTATION_FINAL_PITCH to -5.       // adjust this for different planes' resting pitch.
										// should be slightly lower than resting pitch.
set DEROTATION_BEZIER_STEP_SIZE to 0.2. // to calculate Bezier curve for angular velocity.
										// greater size --> faster derotation.

lock centerline_angular_deviation to (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
lock centerline_linear_deviation to -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.

lock ground_distance_to_runway to latlng(RUNWAY_WEST_THRESHOLD_LAT, RUNWAY_WEST_THRESHOLD_LNG):distance.
lock altitude_above_runway to ship:altitude - RUNWAY_ALTITUDE.
lock distance_to_runway to (ground_distance_to_runway ^ 2 + altitude_above_runway) ^ 0.5.
lock estimated_time_to_deck to distance_to_runway / ship:velocity:surface:mag.

lock localizer to arcsin(centerline_linear_deviation / ground_distance_to_runway).
set AIM_AHEAD_PROPORTION to 3.        // determines amount of lead aim for localizer.
										// 1 is to aim at runway threshold.
										// 2 is to aim at halfway between perpendicular foot to runway centerline, and runway threshold.
										// +infinity is to aim at perpendicular foot to runway centerline.
lock aim_localizer to arctan(AIM_AHEAD_PROPORTION * tan(localizer)).     // aim ahead of the runway by

lock altitude_target to tan(glideslope_target) * ground_distance_to_runway + RUNWAY_ALTITUDE + TARGET_ALTITUDE_INTERCEPT.
lock altitude_error to ship:altitude - altitude_target.

// the following are all vectors, mainly for use in the roll, pitch, and angle of attack calculations
lock rightrotation to ship:facing*r(0,90,0).
lock right to rightrotation:vector. //right and left are directly along wings
lock left to (-1)*right.
lock up to ship:up:vector. //up and down are skyward and groundward
lock down to (-1)*up.
lock fore to ship:facing:vector. //fore and aft point to the nose and tail
lock aft to (-1)*fore.
lock righthor to vcrs(up,fore). //right and left horizons
lock lefthor to (-1)*righthor.
lock forehor to vcrs(righthor,up). //forward and backward horizons
lock afthor to (-1)*forehor.
lock top to vcrs(fore,right). //above the cockpit, through the floor
lock bottom to (-1)*top.
// the following are all angles, useful for control programs
lock absaoa to vang(fore,srfprograde:vector). //absolute angle of attack
lock aoa to vang(top,srfprograde:vector)-90. //pitch component of angle of attack
lock sideslip to vang(right,srfprograde:vector)-90. //yaw component of aoa
lock rollangle to vang(right,righthor)*((90-vang(top,righthor))/abs(90-vang(top,righthor))). // roll angle, 0 at level flight
lock pitchangle to vang(fore,forehor)*((90-vang(fore,up))/abs(90-vang(fore,up))). //pitch angle, 0 at level flight
lock glideslope to vang(srfprograde:vector,forehor)*((90-vang(srfprograde:vector,up))/abs(90-vang(srfprograde:vector,up))).

set pitch_target_pid to pidloop(20, 0.5, 10, -30, 30).                // input: altitude error / altitude target
set pitch_target_pid:setpoint to 0.                                 // output: pitch target.
lock pitch_hold_target to pitch_target_pid:output.
lock pitch_target_error to pitchangle - pitch_hold_target.

set pitch_control_pid to pidloop(0.01, 0.005, 0.003, -1, 1).        // input: pitch target error
set pitch_control_pid to pidloop(0.01, 0.008, 0.008, -1, 1).
set pitch_control_pid:setpoint to 0.

set sideslip_hold_target to 0.
lock sideslip_target_error to sideslip - sideslip_hold_target.
set yaw_control_pid to pidloop(0.005, 0.005, 0.008, -1, 1).         // input: sideslip.
set yaw_control_pid:setpoint to 0.                                  // want sideslip = 0 for coordinated turns

set heading_target_pid to pidloop(2, 0, 1, -50, 50).              // input: aim_localizer. (serves as an error term)
set heading_target_pid:setpoint to 0.                               // output: heading deviation.
lock heading_hold_target to RUNWAY_EASTWARD_HEADING + heading_target_pid:output.         // assumes coming in from the west.

set heading_target_error to 0.
set ship_bearing_rectified to 0.
set roll_hold_target to 0.
lock roll_target_error to rollangle - roll_hold_target.
set roll_target_pid to pidloop(3, 1, 1, -30, 30).        // input: heading error
set roll_target_pid:setpoint to 0.
lock roll_hold_target to roll_target_pid:output.

set roll_control_pid to pidloop(0.003, 0.0005, 0.001, -1, 1).        // input: roll target error
set roll_control_pid:setpoint to 0.

// thrust_pid: input = surface velocity, output = required thrust
lock current_thrust to sum_thrust(engine_list)/ship:maxthrust.
set thrust_pid to pidloop(1, 0.5, 1, 0, 1).
set thrust_pid:setpoint to speed_target.

// throttle_pid: input = required thrust, output = required throttle
set throttle_pid to pidloop(3, 0.5, 1, 0, 1).
set throttle_pid:setpoint to thrust_pid:output.
lock throttle to throttle_pid:output.

set t_last_screen_refresh to time:seconds.
set SCREEN_REFRESH_RATE to 5.
set SCREEN_REFRESH_DT to 1 / SCREEN_REFRESH_RATE.
lock should_refresh to (time:seconds - t_last_screen_refresh > SCREEN_REFRESH_DT).

list engines in engine_list.

function sum_thrust {//.
	parameter engine_list.
	local result is 0.
	for elem in engine_list
	{
		set result to result + elem:thrust.
	}
	return result.
}.

when estimated_time_to_deck < GEAR_DOWN_LEAD_TIME then {
	gear on.
	print "Gear down.".
}

// NEW PROCEDURE
// 0: approach - lineup
// 1: approach - short final (lined up), yaw controls slight heading adjustments
// 2: flare
// 3: rollout

// MAYBE yaw for slight heading adjustments, roll for larger ones

// state 0: approach
set state to 0.
sas off.
until state <> 0 {
	if ship:bearing < 0 {
		set ship_bearing_rectified to -ship:bearing.
	} else {
		set ship_bearing_rectified to 360 - ship:bearing.
	}

	// heading error: bearing - target. Modular math to convert it back to (-180, 180).
	// 1 - turning left is closer: either 0 <= (bearing - target) <= 180 or -360 <= (bearing - target) <= -180.
	//          error: bearing - target.
	//          0 <= (bearing - target) <= 180 || -360 <= (bearing - target) <= -180.
	// 2 - turning right is closer: either 0 <= (target - bearing) <= 180 or -360 <= (target - bearing) <= -180.
	//          error: bearing - target.
	//          -180 <= (bearing - target) <= 0 || 180 <= (bearing - target) <= 360.
	set heading_target_error to (ship_bearing_rectified - heading_hold_target).
	if abs(heading_target_error) > 180 {
		set heading_target_error to -360 * (heading_target_error / abs(heading_target_error)) + heading_target_error.
	}

	set aimpoint to approach_aim_vordme_hdg(RUNWAY_WEST_THRESHOLD_LATLNG, RUNWAY_EASTWARD_HEADING).

	pitch_target_pid:update(time:seconds, altitude_error / altitude_target).
	heading_target_pid:update(time:seconds, aim_localizer).

	thrust_pid:update(time:seconds, ship:velocity:surface:mag).
	set throttle_pid:setpoint to thrust_pid:output.
	throttle_pid:update(time:seconds, current_thrust).

	pitch_control_pid:update(time:seconds, pitch_target_error).
	set ship:control:pitch to pitch_control_pid:output.

	set heading_hold_target to aimpoint:heading.
	if abs(heading_target_error) > ROLL_TARGET_ITERM_HEADING_ERROR_THRESH {
		roll_target_pid:reset.      // only counts I term within [THRESH] degrees of desired heading.
									// prevents heading overshoot.
	}
//	heading_target_pid:update(time:seconds, aim_localizer).
	roll_target_pid:update(time:seconds, heading_target_error).
	roll_control_pid:update(time:seconds, roll_target_error).
	set ship:control:roll to roll_control_pid:output.

	yaw_control_pid:update(time:seconds, sideslip_target_error).
	set ship:control:yaw to yaw_control_pid:output.

	// assuming coming in from the west.
	when altitude_above_runway < FLARE_ALTITUDE_THRESH then {
		set state to 1.
		set vs_before_flare to ship:verticalspeed.
		set roll_hold_target to 0.
		lights on.

		set pitch_target_pid:kP to 12.
		set pitch_target_pid:kI to 7.
		set pitch_target_pid:kD to 2.
		set pitch_target_pid:minoutput to -15.
		set pitch_target_pid:maxoutput to 15.

		set heading_target_pid:kP to 0.5.
		set heading_target_pid:kI to 0.1.
		set heading_target_pid:kD to 0.25.
		set heading_target_pid:minoutput to -0.5.
		set heading_target_pid:maxoutput to 0.5.
	}

	if should_refresh {
		clearscreen.
		print "==== Approach ====".
		print "Pitch target: " + pitch_hold_target.
		print "Pitch error: " + pitch_target_error.
		print "Heading target: " + heading_hold_target.
		print "Roll target: " + roll_hold_target.
		print "-----------".
		print "Altitude target: " + altitude_target.
		print "Altitude error: " + altitude_error.
		print "-----------".
		print "ETA: " + estimated_time_to_deck.
		set t_last_screen_refresh to time:seconds.
	}
}


// state 1: flare
until state <> 1 {

	lock throttle to 0.
	// cubed to transform the bezier curve more densely to the left. Avoid overflare.
	// so that overall v/s is faster and we spend less time at the stable touchdown speed end.
	lock bezier_x to (1 - altitude_above_runway / FLARE_ALTITUDE_THRESH) ^ FLARE_BEZIER_X_POWER.
	lock bezier_y to (1 - bezier_x) ^ 3.
	lock vs_hold_target to bezier_y * (vs_before_flare - TOUCHDOWN_TARGET_VS) + TOUCHDOWN_TARGET_VS.
	lock vs_target_error to ship:verticalspeed - vs_hold_target.

	// heading error and ship bearing from above. should refactor into function.
	if ship:bearing < 0 {
		set ship_bearing_rectified to -ship:bearing.
	} else {
		set ship_bearing_rectified to 360 - ship:bearing.
	}
	set heading_target_error to (ship_bearing_rectified - heading_hold_target).
	if abs(heading_target_error) > 180 {
		set heading_target_error to -360 * (heading_target_error / abs(heading_target_error)) + heading_target_error.
	}

	pitch_target_pid:update(time:seconds, vs_target_error / abs(vs_before_flare)).

	heading_target_pid:update(time:seconds, centerline_linear_deviation).
//    if abs(centerline_linear_deviation) < HEADING_TARGET_ITERM_CENTERLINE_LINEAR_DEVIATION_EPSILON
//    {
//        heading_target_pid:reset.   // resets I term within [EPSILON] m of centerline.
//                                    // stabilizes around centerline.
//    }

	pitch_control_pid:update(time:seconds, pitch_target_error).
	set ship:control:pitch to pitch_control_pid:output.
	roll_control_pid:update(time:seconds, roll_target_error).
	set ship:control:roll to roll_control_pid:output.
	yaw_control_pid:update(time:seconds, heading_target_error).
	set ship:control:yaw to yaw_control_pid:output.

	when ship:status = "landed" then {
		set state to 2.
		brakes on.

		set heading_target_pid:kP to 0.3.
		set heading_target_pid:kI to 0.02.
		set heading_target_pid:kD to 0.04.
		set heading_target_pid:minoutput to -2.
		set heading_target_pid:maxoutput to 2.

		set pitch_before_derotation to pitchangle.
		set d_theta to DEROTATION_FINAL_PITCH - pitch_before_derotation.
	}

	if should_refresh {
		clearscreen.
		print "==== Flare ====".
		print "Pitch target: " + pitch_hold_target.
		print "Pitch error: " + pitch_target_error.
		print "Heading target: " + heading_hold_target.
		print "Roll target: " + roll_hold_target.
		print "V/S error scaled: " + vs_target_error / abs(vs_before_flare).
		print "Altitude above runway: " + altitude_above_runway.
		print "Centerline deviation: " + centerline_linear_deviation.
		set t_last_screen_refresh to time:seconds.
	}
}


// state 2: rollout
until state <> 2 {
	// heading error and ship bearing from above. should refactor into function.
	if ship:bearing < 0 {
		set ship_bearing_rectified to -ship:bearing.
	} else {
		set ship_bearing_rectified to 360 - ship:bearing.
	}
	set heading_target_error to (ship_bearing_rectified - heading_hold_target).
	if abs(heading_target_error) > 180 {
		set heading_target_error to -360 * (heading_target_error / abs(heading_target_error)) + heading_target_error.
	}

	heading_target_pid:update(time:seconds, centerline_linear_deviation).
	yaw_control_pid:update(time:seconds, heading_target_error).
	set ship:control:yaw to yaw_control_pid:output.
//    set ship:control:wheelsteer to yaw_control_pid:output.

	// squared to transform the bezier curve more densely to the left. Quicker derotation at the beginning.
	set bezier_y_step_sign to d_theta / abs(d_theta).
	lock bezier_x to ((pitch_before_derotation - pitchangle) / (pitch_before_derotation - DEROTATION_FINAL_PITCH)) ^ 2.
	lock bezier_y to (1 - bezier_x) ^ 3.
	set pitch_hold_target to pitchangle - bezier_y * DEROTATION_BEZIER_STEP_SIZE * bezier_y_step_sign.

	pitch_control_pid:update(time:seconds, pitch_target_error).
	set ship:control:pitch to pitch_control_pid:output.

	when ship:velocity:surface:mag < WHEEL_STOP_SURFACE_SPEED_EPSILON then {
		set state to 3.
	}

	if should_refresh {
		clearscreen.
		print "==== Rollout ====".
		print "Pitch target: " + pitch_hold_target.
		print "Pitch error: " + pitch_target_error.
		print "Heading target: " + heading_hold_target.
		print "Heading error: " + heading_target_error.
		print "Centerline deviation: " + centerline_linear_deviation.
		set t_last_screen_refresh to time:seconds.
	}
}
