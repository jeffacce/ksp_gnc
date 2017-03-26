set results to list().

when ship:velocity:surface:mag > 1 then {
    results:add(list(ship:geoposition:lat, ship:geoposition:lng)).
    preserve.
}

wait until ship:velocity:surface:mag < 1.

writejson(results, "0:/results.json").
