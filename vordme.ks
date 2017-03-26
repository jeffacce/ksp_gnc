set AIM_AHEAD_PROPORTION to 3.
set VORDME_APPROACH_HDG_ERR_THRESH to 60.

function latlng_away_from_source {//.
	parameter hdg.
	parameter dist.
	parameter source_coords.
	set theta to -hdg + 90.
	if theta < -180 {
		set theta to theta + 360.
	}
	set r to dist / Kerbin:radius.
	set result_lat to source_coords:lat + r * sin(theta) * CONSTANT:RadToDeg.
	set result_lng to source_coords:lng + r * cos(theta) * CONSTANT:RadToDeg.
	if abs(result_lat) > 90 {
		set result_lat to result_lat / abs(result_lat) * 180 - result_lat.
		set result_lng to result_lng + 180.
	}
	if abs(result_lng) > 180 {
		set result_lng to result_lng - result_lng / abs(result_lng) * 360.
	}
	set result to latlng(result_lat, result_lng).
	return result.
}

function approach_aim_vordme_hdg {//.
	parameter station_coords.
	parameter approach_hdg.
	set hdg_from_source to 180 + approach_hdg.
	if hdg_from_source > 360 {
		set hdg_from_source to hdg_from_source - 360.
	}
	set vordme_hdg to station_coords:heading.
	set vordme_dist to station_coords:distance.
	set hdg_err to hdg_diff(vordme_hdg, approach_hdg).
	if abs(hdg_err) > VORDME_APPROACH_HDG_ERR_THRESH {
		return "Angle too big.".
	}
	set dist_on_track to cos(hdg_err) * vordme_dist.
	set dist_aim_from_station to dist_on_track * (1 - 1 / AIM_AHEAD_PROPORTION).
	return latlng_away_from_source(hdg_from_source, dist_aim_from_station, station_coords).
}

function hdg_diff {//.
	parameter hdg_a.
	parameter hdg_b.
	set result to hdg_a - hdg_b.
	if abs(result) > 180 {
		set result to -360 * (result / abs(result)) + result.
	}
	return result.
}

function great_arc_dist {//.
	parameter p1.
	parameter p2.
	set p1_lat to p1:lat * CONSTANT:DegToRad.
	set p1_lng to p1:lng * CONSTANT:DegToRad.
	set p2_lat to p2:lat * CONSTANT:DegToRad.
	set p2_lng to p2:lng * CONSTANT:DegToRad.
	set d_lat to p1_lat - p2_lat.
	set d_lng to p1_lng - p2_lng.
	set a to sin(d_lat/2)^2 + cos(p1_lat) * cos(p2_lat) * sin(d_lng/2)^2.
	set c to 2 * arctan2(sqrt(a), sqrt(1-a)).
	set result to Kerbin:radius * c.
	return result.
}
