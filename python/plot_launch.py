from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import numpy as np
import krpc

# kRPC setup

conn = krpc.connect(name='Plotter', address='127.0.0.1')
vessel = conn.space_center.active_vessel

KCKF = vessel.orbit.body.reference_frame

speed = conn.add_stream(getattr, vessel.flight(KCKF), "speed")
altitude = conn.add_stream(getattr, vessel.flight(KCKF), "mean_altitude")
latitude = conn.add_stream(getattr, vessel.flight(KCKF), "latitude")
longitude = conn.add_stream(getattr, vessel.flight(KCKF), "longitude")
engines = conn.add_stream(getattr, vessel.parts, 'engines')

planet_radius = vessel.orbit.body.equatorial_radius

ksc_launchpad_coords = np.array((-0.09720771, -74.55767342))
ksc_runway_start_coords = np.array((-0.048577539, -74.72581045))
origin = ksc_runway_start_coords

parts_i = len(vessel.parts.all)

# Plot setup

fig = plt.figure()
ax1 = fig.add_subplot(111, aspect='equal')

xlim = 5
ylim = 5

# Gradient

colormap = ListedColormap(np.loadtxt("kerbin_colormap.txt")/255, N=256)  # made with http://jdherman.github.io/colormap/
gradient = np.linspace(0, 10, 256)
Z = np.zeros(shape=(256, 2))
n = 255

for item in gradient:
    Z[n] = np.array([item, item])
    n -= 1

data = pd.DataFrame(columns=["Horizontal distance", "Altitude"])
happenings = pd.DataFrame([[0, -1000, "Launch"]], columns=["Horizontal distance", "Altitude", "Name"])

# EVENTS (include them all as global variables inside animate())

booster_sep = False
MECO = False
# SECO = False


def animate(_):
    global ksc_coord, data, happenings, xlim, ylim, parts_i, booster_sep, MECO, SECO

    position = np.array((latitude(), longitude()))
    this_downrange = (np.linalg.norm(position - origin) * (2.*np.pi*planet_radius)/360.)/1000  # km
    this_altitude = altitude()/1000.0
    this_speed = speed()
    this_active_engines = [_ for _ in engines() if _.active and _.has_fuel]
    this_thrust = sum([_.thrust for _ in this_active_engines]) / 1000.0

    # EVENTS

    parts = len(vessel.parts.all)

    if parts != parts_i and not booster_sep:
        booster_sep = True
        event = pd.DataFrame([[this_downrange, this_altitude, "Booster separation"]],
                             columns=["Horizontal distance", "Altitude", "Name"])
        happenings = happenings.append(event)

    elif parts != parts_i and booster_sep and not MECO:
        MECO = True
        event = pd.DataFrame([[this_downrange, this_altitude, "MECO"]],
                             columns=["Horizontal distance", "Altitude", "Name"])
        happenings = happenings.append(event)

    # elif parts != parts_i and booster_sep and MECO and not SECO:
    #     SECO = True
    #     event = pd.DataFrame([[this_downrange, altitude()/1000, "SECO"]],
    #                          columns=["Horizontal distance", "Altitude", "Name"])
    #     happenings = happenings.append(event)

    parts_i = parts

    # DATA

    newdata = pd.DataFrame([[this_downrange, this_altitude]], columns=["Horizontal distance", "Altitude"])
    data = data.append(newdata)

    xar = data["Horizontal distance"]
    yar = data["Altitude"]
    xhap = happenings["Horizontal distance"]
    yhap = happenings["Altitude"]

    # PLOT

    if xlim - this_downrange < 0.2 * xlim:
        xlim *= 1.2

    if ylim - this_altitude < 0.2 * ylim:
        ylim *= 1.2
        xlim *= 1.2

    ax1.clear()  # can I move this to the start of the loop?
    ax1.plot(xar, yar, color="white", linewidth="3")
    ax1.scatter(xhap, yhap, color="white", linewidth="5")

    plt.xlim(-2, xlim)
    plt.ylim(0, ylim)

    if booster_sep:
        row = happenings.loc[happenings["Name"] == "Booster separation"]
        xy = (row["Horizontal distance"]+xlim*0.1, row["Altitude"]-ylim*0.1)
        xytext = (row["Horizontal distance"]+xlim*0.1, row["Altitude"]+ylim*0.1)
        ax1.annotate("Booster separation", xycoords='data', xy=xy, xytext=xytext, color="white")

    if MECO:
        row = happenings.loc[happenings["Name"] == "MECO"]
        xy = (row["Horizontal distance"]+1, row["Altitude"]-0.5)
        ax1.annotate("MECO", xycoords='data', xy=xy, xytext=xy, color="white")

    # if SECO:
    #     row = happenings.loc[happenings["Name"] == "SECO"]
    #     xy = (row["Horizontal distance"]+1, row["Altitude"]-0.5)
    #     ax1.annotate("SECO", xycoords='data', xy=xy, xytext=xy, color="white")

    ax1.imshow(Z, cmap=colormap, interpolation='bicubic', extent=[-2, xlim, 0, 60])
    ax1.set_axis_bgcolor('black')
    ax1.set_xlabel('Downrange: %.2f km | Velocity: %.2f m/s | Altitude: %.2f km | Thrust: %.2f kN' % (this_downrange, this_speed, this_altitude, this_thrust))
    ax1.set_ylabel('Altitude')

ani = animation.FuncAnimation(fig, animate, interval=200)
plt.show()
