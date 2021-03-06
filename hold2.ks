parameter pitch_hold_target.
parameter roll_hold_target.
parameter sideslip_hold_target.

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

lock localizer to arcsin(centerline_linear_deviation / distance_to_runway).

lock altitude_target to tan(glideslope_target) * distance_to_runway + RUNWAY_ALTITUDE.
lock altitude_error to ship:altitude - altitude_target.

clearscreen.
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


lock pitch_target_error to pitchangle - pitch_hold_target.
lock heading_target_error to ship:heading - heading_hold_target.
lock sideslip_target_error to sideslip - sideslip_hold_target.

set pitch_control_pid to pidloop(0.01, 0.005, 0.003, -1, 1).        // input: pitch target error
set pitch_control_pid:setpoint to 0.                                // want pitch error = 0.

set yaw_control_pid to pidloop(0.005, 0.005, 0.003, -1, 1).         // input: sideslip.
set yaw_control_pid:setpoint to 0.                                  // want sideslip = 0

set roll_control_pid to pidloop(0.005, 0.005, 0.001, -1, 1).        // input: roll omega error
set roll_control_pid:setpoint to 0.                                 // want error = 0.

set roll_target_pid to pidloop(0.05, 0.05, 0.15, -30, 30).          // input: heading error
set roll_target_pid:setpoint to 0.                                  // want error = 0.
// lock roll_hold_target to roll_target_pid:output.
lock roll_target_error to rollangle - roll_hold_target.

set roll_omega_pid to pidloop(0.005, 0.005, 0.015, -200, 200).      // input: roll target error
set roll_omega_pid:setpoint to 0.                                   // want error = 0.
set roll_omega to 0.
lock roll_omega_target to roll_omega_pid:output.
lock roll_omega_target_error to roll_omega - roll_omega_target.

set last_rollangle to rollangle.
set last_right to right.
set last_timestamp to time:seconds.


// state 0: approach
set state to 0.
sas off.
until state <> 0 {

    pitch_control_pid:update(time:seconds, pitch_target_error).
    set ship:control:pitch to pitch_control_pid:output.

    // roll control --> roll omega --> roll target (--> heading omega) --> heading target
    // roll_target_pid:update(time:seconds, heading_target_error).
    roll_omega_pid:update(time:seconds, -roll_target_error).
    roll_control_pid:update(time:seconds, roll_omega_target_error).
    set ship:control:roll to roll_control_pid:output.

    yaw_control_pid:update(time:seconds, sideslip_target_error).
    set ship:control:yaw to yaw_control_pid:output.

    // PROBLEM: -180, 180 at upside down gives bad angular velocity.
    // Should use vector math instead of rollangle.
    if (rollangle < -180 or rollangle > 180) {
        set rollangle_for_calc to 180.  // bound bad rollangles near upside down.
    } else {
        set rollangle_for_calc to rollangle.
    }
    if (rollangle_for_calc > 100 and last_rollangle < -100) {
        set roll_omega to (rollangle_for_calc - 360 - last_rollangle) / (time:seconds - last_timestamp).
    } else if (rollangle_for_calc < -100 and last_rollangle > 100) {
        set roll_omega to (rollangle_for_calc + 360 - last_rollangle) / (time:seconds - last_timestamp).
    } else {
        set roll_omega to (rollangle_for_calc - last_rollangle) / (time:seconds - last_timestamp).
    }
    set last_rollangle to rollangle_for_calc.
    set last_timestamp to time:seconds.


    clearscreen.
    print "Pitch target: " + pitch_hold_target.
    print "Pitch error: " + pitch_target_error.
    print "Sideslip target: " + sideslip_hold_target.
    print "Sideslip: " + sideslip.
//    print "Heading target: " + heading_hold_target.
//    print "Heading error: " + heading_target_error.
//    print "Roll omega target: " + roll_omega_target.
//    print "Roll omega error: " + roll_omega_target_error.
    print "Roll target: " + roll_hold_target.
    print "Roll error: " + roll_target_error.
    print "Roll control output: " + roll_control_pid:output.
}

wait until altitude_above_runway < 20.

// state 1: flare



// state 2: rollout