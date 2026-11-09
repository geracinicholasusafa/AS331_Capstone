"""
Makes the team's four TCS graphs from an SD card log (TCS_log_dataNNN.csv).

    python plot_tcs_log.py TCS_log_data005.csv                 # whole run
    python plot_tcs_log.py TCS_log_data005.csv --start 60 --end 600   # only 60 s to 600 s
    python plot_tcs_log.py TCS_log_data005.csv --out my_plots

Needs pandas and matplotlib (pip install pandas matplotlib).
Writes four PNG files:
    1_sun_direction.png     the satellite knows where the light is
    2_temperature.png       temperature over time
    3_pointing.png          finding the light and holding the louvers away from it (gyro + reaction wheel)
    4_temperature_hold.png  heating up to the hold temperature, then the louvers holding it there
Every graph uses only one y-scale per panel; different quantities get their own panel on a shared time axis.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

# Colors in a fixed order (a colorblind-checked palette), plus neutral inks.
BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
STATE_SHADE = {"SUN_AVOID": "#fbe3d6", "COOLING": "#d9e8f8", "FAULT": "#f6d4d4"}

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "font.size": 10, "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlecolor": INK,
    "legend.frameon": False, "lines.linewidth": 2,
})


def shade_states(ax, df):
    """Light background bands showing when the TCS was in SUN_AVOID, COOLING or FAULT."""
    start = None
    for i in range(len(df)):
        s = df.state.iat[i]
        if start is None or s != df.state.iat[start]:
            if start is not None:
                _band(ax, df, start, i)
            start = i
    if start is not None:
        _band(ax, df, start, len(df) - 1)


def _band(ax, df, a, b):
    color = STATE_SHADE.get(df.state.iat[a])
    if color:
        ax.axvspan(df.elapsed_s.iat[a], df.elapsed_s.iat[b], color=color, lw=0, zorder=0)


def state_legend(ax):
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in STATE_SHADE.values()]
    return handles, list(STATE_SHADE.keys())


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


def graph_sun(df, out):
    fig, (a, b) = plt.subplots(2, 1, figsize=(11, 7), sharex=True, height_ratios=[3, 2])
    a.scatter(df.elapsed_s, df.light_dir_deg, s=8, color=BLUE, label="Light direction seen by the satellite")
    if "sun_in_room_deg" in df:
        a.scatter(df.elapsed_s, df.sun_in_room_deg, s=8, color=ORANGE,
                  label="Light direction in the room (seen + gyro heading): flat = satellite knows where the light is")
    a.set_ylim(0, 360); a.set_yticks([0, 90, 180, 270, 360])
    a.set_ylabel("Direction (deg)\n0=+X 90=+Y 180=-X 270=-Y")
    a.set_title("1. The satellite knows where the light is")
    a.legend(loc="lower left", bbox_to_anchor=(0, 1.0), fontsize=9, markerscale=2)
    a.set_title(a.get_title(), pad=44)
    for col, color, name in [("light_px", BLUE, "+X"), ("light_mx", ORANGE, "-X"), ("light_py", AQUA, "+Y"), ("light_my", YELLOW, "-Y")]:
        if col in df:
            b.plot(df.elapsed_s, df[col], color=color, lw=1.5, label=f"{name} sensor")
    b.set_ylabel("Light sensor (counts)")
    b.set_xlabel("Time since logging started (s)")
    b.legend(loc="lower left", bbox_to_anchor=(0, 1.0), ncol=4, fontsize=9)
    save(fig, out, "1_sun_direction.png")


def graph_temperature(df, out):
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(df.elapsed_s, df.temp1_C, color=BLUE, label="Thermocouple 1 (panel, used for control)")
    ax.plot(df.elapsed_s, df.temp2_C, color=ORANGE, label="Thermocouple 2")
    ax.set_ylabel("Temperature (C)")
    ax.set_xlabel("Time since logging started (s)")
    ax.set_title("2. Temperature over time")
    ax.legend(loc="lower left", bbox_to_anchor=(0, 1.0), ncol=2, fontsize=9)
    ax.set_title(ax.get_title(), pad=26)
    save(fig, out, "2_temperature.png")


def graph_pointing(df, out):
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    a, b, c, d = axes
    for ax in axes:
        shade_states(ax, df)
    a.axhspan(-15, 15, color="#dcefe6", lw=0, zorder=0)
    a.scatter(df.elapsed_s, df.vent_error_deg, s=8, color=BLUE)
    a.set_ylim(-180, 180); a.set_yticks([-180, -90, 0, 90, 180])
    a.set_ylabel("Louvers off from\n'away from light' (deg)")
    a.set_title("3. Finding the light and holding the louvers away from it (gyro + reaction wheel)")
    h, l = state_legend(a)
    h.append(plt.Rectangle((0, 0), 1, 1, color="#dcefe6")); l.append("Aimed (within 15 deg)")
    a.legend(h, l, loc="lower left", bbox_to_anchor=(0, 1.0), ncol=4, fontsize=8)
    a.set_title(a.get_title(), pad=26)
    b.plot(df.elapsed_s, df.heading_deg, color=BLUE)
    b.set_ylabel("Heading from gyro\n(deg, since log start)")
    c.plot(df.elapsed_s, df.spin_deg_per_s, color=BLUE, lw=1.2)
    c.set_ylabel("Spin speed\n(deg/s)")
    d.plot(df.elapsed_s, df.wheel_rpm, color=BLUE, lw=1.2)
    d.axhline(400, color=INK2, lw=1, ls="--"); d.axhline(-400, color=INK2, lw=1, ls="--")
    d.set_ylabel("Reaction wheel\n(RPM, limit 400)")
    d.set_xlabel("Time since logging started (s)")
    save(fig, out, "3_pointing.png")


def graph_hold(df, out):
    fig, (a, b) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for ax in (a, b):
        shade_states(ax, df)
    a.plot(df.elapsed_s, df.temp1_C, color=BLUE, label="Panel temperature")
    a.plot(df.elapsed_s, df.hold_C, color=INK2, lw=1.5, ls="--", label="Hold temperature")
    a.set_ylabel("Temperature (C)")
    a.set_title("4. Heating to the hold temperature, then the louvers holding it there")
    h, l = a.get_legend_handles_labels()
    sh, sl = state_legend(a)
    a.legend(h + sh, l + sl, loc="lower left", bbox_to_anchor=(0, 1.0), ncol=5, fontsize=8)
    a.set_title(a.get_title(), pad=26)
    b.plot(df.elapsed_s, df.louver_pct, color=BLUE, label="Louver opening")
    b.plot(df.elapsed_s, df.peltier_pct, color=ORANGE, label="Heater power")
    b.set_ylim(-5, 105)
    b.set_ylabel("Percent")
    b.set_xlabel("Time since logging started (s)")
    b.legend(loc="lower left", bbox_to_anchor=(0, 1.0), ncol=2, fontsize=9)
    save(fig, out, "4_temperature_hold.png")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv")
    p.add_argument("--start", type=float, default=None, help="first second to plot")
    p.add_argument("--end", type=float, default=None, help="last second to plot")
    p.add_argument("--out", default=None, help="folder for the PNGs (default: <csv name>_plots next to the CSV)")
    args = p.parse_args()

    df = pd.read_csv(args.csv, skipinitialspace=True)
    if args.start is not None:
        df = df[df.elapsed_s >= args.start]
    if args.end is not None:
        df = df[df.elapsed_s <= args.end]
    df = df.reset_index(drop=True)
    out = args.out or os.path.splitext(args.csv)[0] + "_plots"
    os.makedirs(out, exist_ok=True)

    graph_sun(df, out)
    graph_temperature(df, out)
    if "heading_deg" in df:
        graph_pointing(df, out)
    else:
        print("skipped 3_pointing.png: this log is from older firmware without gyro columns")
    graph_hold(df, out)


if __name__ == "__main__":
    main()
