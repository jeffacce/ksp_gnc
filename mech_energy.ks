set t_last_screen_refresh to time:seconds.
set SCREEN_REFRESH_RATE to 10.
set SCREEN_REFRESH_DT to 1 / SCREEN_REFRESH_RATE.
lock should_refresh to (time:seconds - t_last_screen_refresh > SCREEN_REFRESH_DT).

lock E_grav to -constant:G * Kerbin:mass * ship:mass / (ship:altitude + Kerbin:radius).
lock E_grav_surf_ref to -constant:G * Kerbin:mass * ship:mass / Kerbin:radius.
lock E_grav_normed to E_grav - E_grav_surf_ref.

lock E_k_surf to 0.5 * ship:mass * ship:velocity:surface:mag ^ 2.

lock E_mech to E_grav_normed + E_k_surf.
set last_timestamp to time:seconds - 0.01.
set last_E_mech to E_mech.
// P = dW/dt = dE/dt.
lock power to (E_mech - last_E_mech) / (time:seconds - last_timestamp).
// F = P/v.
lock net_force to power / ship:velocity:surface:mag.

until False {

	if should_refresh {
		clearscreen.
		print "Gravitational: " + E_grav_normed + "J".
		print "      Kinetic: " + E_k_surf + "J".
		print "   Mechanical: " + E_mech + "J".
		print "        Power: " + power + "W".
		print "    Net force: " + net_force + "N".
		set t_last_screen_refresh to time:seconds.
	}
}