parameter glideslope_target.
parameter speed_target.

// PID flow:
// raw pitch control <-- pitch target control <-- altitude error = 0
// raw roll control <-- roll target control <-- heading target control <-- localizer error = 0
// raw yaw control <-- sideslip = 0

set CENTERLINE_EQ_A to 142.13236295.    // latitude coefficient for the linear equation
set CENTERLINE_EQ_B to 1.               // longitude coefficient
set CENTERLINE_EQ_C to 81.62849024.     // constant term
set RUNWAY_WEST_THRESHOLD_LAT to -0.0485997.
set RUNWAY_WEST_THRESHOLD_LNG to -74.724375.
set RUNWAY_EAST_THRESHOLD_LAT to -0.0502118560109606.
set RUNWAY_EAST_THRESHOLD_LNG to -74.4899977802028.
set RUNWAY_ALTITUDE to 70.28267.
set ROLL_TARGET_ITERM_HEADING_ERROR_THRESH to 5.
set PITCH_TARGET_ITERM_ALTITUDE_ERROR_THRESH to 20.
set HEADING_TARGET_ITERM_LOCALIZER_THRESH to 0.3.
set FLARE_ALTITUDE_THRESH to 30.
set FLARE_STABILIZE_ALTITUDE_THRESH to 10.
set TOUCHDOWN_TARGET_VS to -1.          // desired touchdown vertical speed: -1m/s
set TOUCHDOWN_STATE_ALTITUDE_EPSILON to 1.
set HEADING_TARGET_ITERM_CENTERLINE_LINEAR_DEVIATION_EPSILON to 0.5.

lock centerline_angular_deviation to (CENTERLINE_EQ_A * ship:geoposition:lat + CENTERLINE_EQ_B * ship:geoposition:lng + CENTERLINE_EQ_C) / sqrt(CENTERLINE_EQ_A^2 + CENTERLINE_EQ_B^2).
lock centerline_linear_deviation to -2 * constant:pi * KERBIN:RADIUS * centerline_angular_deviation / 360.

lock distance_to_runway to latlng(RUNWAY_WEST_THRESHOLD_LAT, RUNWAY_WEST_THRESHOLD_LNG):distance.
lock altitude_above_runway to ship:altitude - RUNWAY_ALTITUDE.

lock localizer to arcsin(centerline_linear_deviation / distance_to_runway).

lock altitude_target to tan(glideslope_target) * distance_to_runway + RUNWAY_ALTITUDE.
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


set pitch_target_pid to pidloop(60, 5, 10, -30, 30).                // input: altitude error / altitude target
set pitch_target_pid:setpoint to 0.                                 // output: pitch target.
lock pitch_hold_target to pitch_target_pid:output.
lock pitch_target_error to pitchangle - pitch_hold_target.

set pitch_control_pid to pidloop(0.01, 0.005, 0.003, -1, 1).        // input: pitch target error
set pitch_control_pid:setpoint to 0.

set sideslip_hold_target to 0.
lock sideslip_target_error to sideslip - sideslip_hold_target.
set yaw_control_pid to pidloop(0.005, 0.005, 0.008, -1, 1).         // input: sideslip.
set yaw_control_pid:setpoint to 0.                                  // want sideslip = 0 for coordinated turns

set heading_target_pid to pidloop(3, 0.01, 1, -50, 50).              // input: localizer. (serves as an error term)
set heading_target_pid:setpoint to 0.                               // output: heading deviation.
lock heading_hold_target to 90 + heading_target_pid:output.         // 90 assumes coming in from the west.

set heading_target_error to 0.
set ship_bearing_rectified to 0.
set roll_hold_target to 0.
lock roll_target_error to rollangle - roll_hold_target.
set roll_target_pid to pidloop(3, 0.05, 0.5, -30, 30).        // input: heading error
set roll_target_pid:setpoint to 0.
lock roll_hold_target to roll_target_pid:output.

set roll_control_pid to pidloop(0.005, 0.001, 0.002, -1, 1).        // input: roll target error
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

list engines in engine_list.

function sum_thrust {
    parameter engine_list.
    local result is 0.
    for elem in engine_list {
        set result to result + elem:thrust.
    }
    return result.
}.

// state 0: approach
set state to 0.
sas off.
until state > 1 {

    set should_refresh to (time:seconds - t_last_screen_refresh > SCREEN_REFRESH_DT).

    if should_refresh {
        clearscreen.
        print "Pitch target: " + pitch_hold_target.
        print "Heading target: " + heading_hold_target.
        print "Roll target: " + roll_hold_target.
        print "-----------".
    }

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

    if state = 0 {
        pitch_target_pid:update(time:seconds, altitude_error / altitude_target).
        if abs(altitude_error) > PITCH_TARGET_ITERM_ALTITUDE_ERROR_THRESH {
            pitch_target_pid:reset.     // only counts I term within [THRESH] meters of desired altitude.
                                        // prevents altitude overshoot.
        }

        heading_target_pid:update(time:seconds, localizer).
        if abs(localizer) > HEADING_TARGET_ITERM_LOCALIZER_THRESH {
            heading_target_pid:reset.   // only counts I term within [THRESH] degrees of desired localizer (normally 0).
                                        // prevents localizer overshoot.
        }

        if should_refresh {
            print "Altitude target: " + altitude_target.
            print "Altitude error: " + altitude_error.
            print "-----------".
        }

        thrust_pid:update(time:seconds, ship:velocity:surface:mag).
        set throttle_pid:setpoint to thrust_pid:output.
        throttle_pid:update(time:seconds, current_thrust).

    } else if state = 1 {
        pitch_target_pid:update(time:seconds, vs_target_error / abs(vs_before_flare)).

        heading_target_pid:update(time:seconds, -centerline_linear_deviation).
        if abs(centerline_linear_deviation) < HEADING_TARGET_ITERM_CENTERLINE_LINEAR_DEVIATION_EPSILON {
            heading_target_pid:reset.   // resets I term within [EPSILON] m of centerline.
                                        // stabilizes around centerline.
        }

        if should_refresh {
            print "Pitch Target kP: " + pitch_target_pid:kp.
            print "Altitude above runway: " + altitude_above_runway.
            print "V/S target: " + vs_hold_target.
            print "V/S error scaled: " + vs_target_error / abs(vs_before_flare).
            print "Centerline deviation: " + centerline_linear_deviation.
        }
    }

    if should_refresh {
        set t_last_screen_refresh to time:seconds.
    }

    pitch_control_pid:update(time:seconds, pitch_target_error).
    set ship:control:pitch to pitch_control_pid:output.

    if abs(heading_target_error) > ROLL_TARGET_ITERM_HEADING_ERROR_THRESH {
        roll_target_pid:reset.      // only counts I term within [THRESH] degrees of desired heading.
                                    // prevents heading overshoot.
    }


    heading_target_pid:update(time:seconds, localizer).
    roll_target_pid:update(time:seconds, heading_target_error).
    roll_control_pid:update(time:seconds, roll_target_error).
    set ship:control:roll to roll_control_pid:output.

    yaw_control_pid:update(time:seconds, sideslip_target_error).
    set ship:control:yaw to yaw_control_pid:output.


    when altitude_above_runway < FLARE_ALTITUDE_THRESH then {
        set state to 1.
        set vs_before_flare to ship:verticalspeed.
        set pitch_target_pid to pidloop(20, 5, 10, -15, 15).
        set pitch_target_pid:setpoint to 0.
        lock throttle to 0.
        lock pitch_hold_target to pitch_target_pid:output.
        lock pitch_target_error to pitchangle - pitch_hold_target.
        // squared to transform the bezier curve more densely to the left. Avoid overflare.
        // so that overall v/s is faster and we spend less time at the stable touchdown speed end.
        lock bezier_x to (1 - altitude_above_runway / FLARE_ALTITUDE_THRESH) ^ 2.
        lock bezier_y to (1 - bezier_x) ^ 3.
        lock vs_hold_target to bezier_y * (vs_before_flare - TOUCHDOWN_TARGET_VS) + TOUCHDOWN_TARGET_VS.
        lock vs_target_error to ship:verticalspeed - vs_hold_target.

        set heading_target_pid to pidloop(0.01, 0.005, 0.005, -2, 2).
    }

    when altitude_above_runway < FLARE_STABILIZE_ALTITUDE_THRESH then {
        unlock pitch_hold_target.
        set pitch_hold_target to pitch_target_pid:output.
        set roll_hold_target to 0.
    }

    when abs(altitude_above_runway) < TOUCHDOWN_STATE_ALTITUDE_EPSILON or ship:status = "landed" then {
        set state to 2.
    }

}

//


// state 2: rollout