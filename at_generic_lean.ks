parameter speed_target.

set THRUST_PID_ITERM_SPEED_ERROR_THRESH to 10.

if ship:maxthrust=0 {
	print "Ship has no usable thrust. Do you have a working engine?".
}

list engines in engine_list.

function sum_thrust {//.
	parameter engine_list.
	local result is 0.
	for elem in engine_list {//.
		set result to result + elem:thrust.
	}
	return result.
}.

lock speed_error to ship:velocity:surface:mag - speed_target.
set thrust_pid to pidloop(0.05, 0.01, 0.03, 0, 1).
set thrust_pid:setpoint to 0.
lock thrust_target to thrust_pid:output.

lock thrust_error to sum_thrust(engine_list)/ship:maxthrust - thrust_target.
set throttle_pid to pidloop(10, 2, 1, 0, 1).
set throttle_pid:setpoint to 0.
lock throttle to thrust_target + throttle_pid:output * (1 - thrust_target).

set t_last_screen_refresh to time:seconds.
set SCREEN_REFRESH_RATE to 5.
set SCREEN_REFRESH_DT to 1 / SCREEN_REFRESH_RATE.
lock should_refresh to (time:seconds - t_last_screen_refresh > SCREEN_REFRESH_DT).

until False {
	if abs(speed_error) > THRUST_PID_ITERM_SPEED_ERROR_THRESH {
		thrust_pid:reset.
	}
	thrust_pid:update(time:seconds, speed_error).
	throttle_pid:update(time:seconds, thrust_error).
	if should_refresh {
		clearscreen.
		print "Speed error: " + speed_error.
		print "Thrust target: " + thrust_target.
		print "-----------------------------".
		print "Thrust error: " + thrust_error.
		print "Throttle: " + throttle.
		set t_last_screen_refresh to time:seconds.
	}

}
