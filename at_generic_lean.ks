parameter speed_target.

list engines in engine_list.

function sum_thrust {
  parameter engine_list.
  local result is 0.
  for elem in engine_list {
    set result to result + elem:thrust.
  }
  return result.
}.

// thrust_pid: input = surface velocity, output = required thrust

if ship:maxthrust=0 {
    print "Ship has no usable thrust. Do you have a working engine?".
}

lock current_thrust to sum_thrust(engine_list)/ship:maxthrust.
set thrust_pid to pidloop(1, 0.5, 1, 0, 1).
set thrust_pid:setpoint to speed_target.

// throttle_pid: input = required thrust, output = required throttle
set throttle_pid to pidloop(3, 0.5, 1, 0, 1).
set throttle_pid:setpoint to thrust_pid:output.
lock throttle to throttle_pid:output.

when True then {
    thrust_pid:update(time:seconds, ship:velocity:surface:mag).
    set throttle_pid:setpoint to thrust_pid:output.
    throttle_pid:update(time:seconds, current_thrust).
}

wait until False.