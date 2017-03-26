run vordme.
runpath("/library/lib_list_dialog").
runpath("/library/lib_number_dialog").

set speed_hold_target to ship:velocity:surface:mag.
set altitude_hold_target to ship:altitude.
if ship:bearing < 0 {
	set ship_bearing_rectified to -ship:bearing.
} else {
	set ship_bearing_rectified to 360 - ship:bearing.
}
set heading_hold_target to ship_bearing_rectified.
lock altitude_target_error to ship:altitude - altitude_hold_target.
set PITCH_TARGET_ITERM_ALTITUDE_ERROR_PROPORTION_THRESH to 0.05.
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

set pitch_target_pid to pidloop(20, 2, 10, -30, 30).                // input: altitude error / altitude target
set pitch_target_pid:setpoint to 0.                                 // output: pitch target.
lock pitch_hold_target to pitch_target_pid:output.
lock pitch_target_error to pitchangle - pitch_hold_target.

set pitch_control_pid to pidloop(0.032, 0.008, 0.004, -1, 1).
set pitch_control_pid:setpoint to 0.

set sideslip_hold_target to 0.
lock sideslip_target_error to sideslip - sideslip_hold_target.
set yaw_control_pid to pidloop(0.005, 0.005, 0.008, -1, 1).         // input: sideslip.
set yaw_control_pid:setpoint to 0.                                  // want sideslip = 0 for coordinated turns

set heading_target_pid to pidloop(2, 0, 0.5, -50, 50).              // input: aim_localizer. (serves as an error term)
set heading_target_pid:setpoint to 0.                               // output: heading deviation.
set heading_target_error to 0.

set roll_hold_target to 0.
lock roll_target_error to rollangle - roll_hold_target.
set roll_target_pid to pidloop(3, 0.05, 0.5, -30, 30).        // input: heading error
set roll_target_pid:setpoint to 0.
lock roll_hold_target to roll_target_pid:output.

set roll_control_pid to pidloop(0.006, 0.001, 0.002, -1, 1).        // input: roll target error
set roll_control_pid:setpoint to 0.

// thrust_pid: input = surface velocity, output = required thrust
lock current_thrust to sum_thrust(engine_list)/ship:maxthrust.
set thrust_pid to pidloop(1, 0.5, 1, 0, 1).
set thrust_pid:setpoint to speed_hold_target.

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

set autopilot_power to 1.
sas off.
set display_menu to False.
set choice to -1.
until autopilot_power <> 1 {
	if ship:bearing < 0 {
		set ship_bearing_rectified to -ship:bearing.
	} else {
		set ship_bearing_rectified to 360 - ship:bearing.
	}
	set heading_target_error to (ship_bearing_rectified - heading_hold_target).
	if abs(heading_target_error) > 180 {
		set heading_target_error to -360 * (heading_target_error / abs(heading_target_error)) + heading_target_error.
	}

	pitch_target_pid:update(time:seconds, altitude_target_error / altitude_hold_target).
	pitch_control_pid:update(time:seconds, pitch_target_error).
	set ship:control:pitch to pitch_control_pid:output.

	thrust_pid:update(time:seconds, ship:velocity:surface:mag).
	set throttle_pid:setpoint to thrust_pid:output.
	throttle_pid:update(time:seconds, current_thrust).

	if abs(heading_target_error) > ROLL_TARGET_ITERM_HEADING_ERROR_THRESH {
		roll_target_pid:reset.      // only counts I term within [THRESH] degrees of desired heading.
									// prevents heading overshoot.
	}

	roll_target_pid:update(time:seconds, heading_target_error).
	roll_control_pid:update(time:seconds, roll_target_error).
	set ship:control:roll to roll_control_pid:output.

	yaw_control_pid:update(time:seconds, sideslip_target_error).
	set ship:control:yaw to yaw_control_pid:output.

	when ag5 then {
		set ag5 to False.
		set display_menu to True.
		preserve.
	}

	if display_menu {
		set display_menu to False.
		set choice to open_cancelable_list_dialog("Autopilot Menu", list("Adjust Heading", "Adjust Altitude", "Adjust Speed")).
	}

	if choice > -1 {
		if choice = 0 {
			set heading_hold_target to open_number_dialog("Adjust Heading", heading_hold_target).
		}
		if choice = 1 {
			set altitude_hold_target to open_number_dialog("Adjust Altitude", altitude_hold_target).
		}
		if choice = 2 {
			set speed_hold_target to open_number_dialog("Adjust Speed", speed_hold_target).
		}
		set choice to -1.
	}


	if should_refresh {
		clearscreen.
		print "Speed target: " + speed_hold_target.
		print "-----------".
		print "Pitch target: " + pitch_hold_target.
		print "Heading target: " + heading_hold_target.
		print "Roll target: " + roll_hold_target.
		print "-----------".
		print "Altitude target: " + altitude_hold_target.
		print "Altitude error: " + altitude_target_error.
		set t_last_screen_refresh to time:seconds.
	}
}
