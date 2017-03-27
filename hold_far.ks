parameter pitch_hold_target.
parameter heading_hold_target.
parameter sideslip_hold_target.


set ROLL_TARGET_ITERM_HEADING_ERROR_THRESH to 2.

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


set KERBIN_SCALE_HEIGHT to 5000.
lock current_atm to constant:e ^ (-altitude / KERBIN_SCALE_HEIGHT).
lock flow_v to cos(absaoa) * ship:velocity:surface:mag.
set GAIN_SCHEDULE_REF to 11476.     // cos(AoA) * rho * v^2, at 5 degrees AoA, 0.8atm, 120m/s
lock gain_schedule_scale to GAIN_SCHEDULE_REF / (current_atm * flow_v^2).
// TEST: 10000m (0.13atm), 200m/s
// TEST: 20000m (0.018atm), 1400m/s
// TEST: 5000m (0.37atm), 150m/s
// TEST: 2000m (0.67atm), 130m/s, flaps up/down
// Hypothesis: solving for flow_v here gives us approximate stall speed
// (because the reference point was calculated from flaps full, landing speed)

lock pitch_target_error to pitchangle - pitch_hold_target.
set heading_target_error to 0.
set ship_bearing_rectified to 0.
lock sideslip_target_error to sideslip - sideslip_hold_target.

set PITCH_CTRL_INIT_KP to 0.048.
set PITCH_CTRL_INIT_KI to 0.016.
set PITCH_CTRL_INIT_KD to 0.008.
set pitch_control_pid to pidloop(PITCH_CTRL_INIT_KP, PITCH_CTRL_INIT_KI, PITCH_CTRL_INIT_KD, -1, 1).
set pitch_control_pid:setpoint to 0.

set YAW_CTRL_INIT_KP to 0.005.
set YAW_CTRL_INIT_KI to 0.005.
set YAW_CTRL_INIT_KD to 0.008.
set yaw_control_pid to pidloop(YAW_CTRL_INIT_KP, YAW_CTRL_INIT_KI, YAW_CTRL_INIT_KD, -1, 1).         // input: sideslip.
set yaw_control_pid:setpoint to 0.                                  // want sideslip = 0 for coordinated turns

set ROLL_CTRL_INIT_KP to 0.012.
set ROLL_CTRL_INIT_KI to 0.002.
set ROLL_CTRL_INIT_KD to 0.004.
set roll_control_pid to pidloop(ROLL_CTRL_INIT_KP, ROLL_CTRL_INIT_KI, ROLL_CTRL_INIT_KD, -1, 1).        // input: roll target error
set roll_control_pid:setpoint to 0.

set heading_target_error to 0.
set ship_bearing_rectified to 0.
set roll_hold_target to 0.
lock roll_target_error to rollangle - roll_hold_target.
set roll_target_pid to pidloop(3, 0, 1, -30, 30).        // input: heading error
set roll_target_pid:setpoint to 0.
lock roll_hold_target to roll_target_pid:output.

set roll_hold_target to 0.
lock roll_target_error to rollangle - roll_hold_target.
set roll_target_pid to pidloop(1.5, 0.05, 0.5, -30, 30).        // input: heading error
set roll_target_pid:setpoint to 0.                                  // want heading error = 0.
lock roll_hold_target to roll_target_pid:output.

set t_last_screen_refresh to time:seconds.
set SCREEN_REFRESH_RATE to 5.
set SCREEN_REFRESH_DT to 1 / SCREEN_REFRESH_RATE.
lock should_refresh to (time:seconds - t_last_screen_refresh > SCREEN_REFRESH_DT).

set state to 0.
sas off.
until sas {

    set pitch_control_pid:kP to PITCH_CTRL_INIT_KP * gain_schedule_scale.
    set roll_control_pid:kP to ROLL_CTRL_INIT_KP * gain_schedule_scale.
    set yaw_control_pid:kP to YAW_CTRL_INIT_KP * gain_schedule_scale.

    if ship:bearing < 0 {
        set ship_bearing_rectified to -ship:bearing.
    } else {
        set ship_bearing_rectified to 360 - ship:bearing.
    }

    set heading_target_error to (ship_bearing_rectified - heading_hold_target).
    if abs(heading_target_error) > 180 {
        set heading_target_error to -360 * (heading_target_error / abs(heading_target_error)) + heading_target_error.
    }

    pitch_control_pid:update(time:seconds, pitch_target_error).
    set ship:control:pitch to pitch_control_pid:output.

    if abs(heading_target_error) > ROLL_TARGET_ITERM_HEADING_ERROR_THRESH {
        roll_target_pid:reset.      // reset the integral term to stabilize at desired heading.
    }
    roll_target_pid:update(time:seconds, heading_target_error).
    roll_control_pid:update(time:seconds, roll_target_error).
    set ship:control:roll to roll_control_pid:output.

    yaw_control_pid:update(time:seconds, sideslip_target_error).
    set ship:control:yaw to yaw_control_pid:output.


    if should_refresh {
        clearscreen.
        print "Pitch target: " + pitch_hold_target.
        print "Pitch error: " + pitch_target_error.
        print "-------------------------------".
        print "Heading target: " + heading_hold_target.
        print "Roll target: " + roll_hold_target.
        print "Roll error: " + roll_target_error.
        print "-------------------------------".
        print "Sideslip target: " + sideslip_hold_target.
        print "Sideslip error: " + sideslip_target_error.
        print "-------------------------------".
        print "Gain schedule scale: " + gain_schedule_scale.
        set t_last_screen_refresh to time:seconds.
    }
}
