"""One-page summary figure for a Report."""

import os

import numpy as np

from .doctor import in_segments, local_xy

MODE_COLOURS = ["#dbeafe", "#dcfce7", "#fef3c7", "#fce7f3", "#ede9fe", "#ffedd5", "#e0f2fe", "#f1f5f9"]


def figure(r, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    log = r.log
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(13, 8.6), dpi=120)
    g = fig.add_gridspec(3, 3, width_ratios=[1, 1, 0.9], hspace=0.5, wspace=0.35)
    ax_alt, ax_vib, ax_att = (fig.add_subplot(g[i, :2]) for i in range(3))
    ax_trk, ax_bat, ax_ekf = (fig.add_subplot(g[i, 2]) for i in range(3))
    end = max(r.duration_s, 1)

    names = []
    for a, b, name in r.modes:
        if name not in names:
            names.append(name)
    for ax in (ax_alt, ax_vib, ax_att):
        for a, b, name in r.modes:
            ax.axvspan(a, b, color=MODE_COLOURS[names.index(name) % len(MODE_COLOURS)], lw=0, zorder=0)
        for a, b in r.flights:
            ax.axvline(b, color="#ef4444", lw=0.7, ls="--", zorder=1)
        ax.set_xlim(0, end)
        ax.grid(alpha=0.25)
    for a, b, name in r.modes:
        if (b - a) / end > 0.06:
            ax_alt.text((a + b) / 2, 1.02, name, transform=ax_alt.get_xaxis_transform(), ha="center", fontsize=7.5, weight="bold", color="#374151")

    pos, baro = log.get("POS"), log.get("BARO")
    if pos is not None and "RelHomeAlt" in pos:
        ax_alt.plot(pos["t"], pos["RelHomeAlt"], color="#2563eb", lw=1.1)
    elif baro is not None:
        ax_alt.plot(baro["t"], baro["Alt"], color="#2563eb", lw=1.1)
    ax_alt.set_ylabel("Altitude (m)")

    v = log.get("VIBE", 0)
    if v is not None:
        for k, c in (("VibeX", "#2563eb"), ("VibeY", "#f97316"), ("VibeZ", "#16a34a")):
            ax_vib.plot(v["t"], v[k], lw=0.7, color=c, label=k[-1])
        clip = v.get("Clip", v.get("Clip0"))
        if clip is not None:
            inc = np.diff(clip, prepend=clip[0]) > 0
            if inc.any():
                ax_vib.plot(v["t"][inc], np.full(inc.sum(), max(5, np.nanmax(v["VibeZ"]) * 0.9)), "v", color="#ef4444", ms=5, label="clip")
        ax_vib.axhline(30, color="#ef4444", lw=0.6, ls=":")
        ax_vib.legend(loc="upper left", fontsize=7, frameon=False, ncol=4)
    ax_vib.set_ylabel("Vibration (m/s²)")

    att = log.get("ATT")
    if att is not None:
        ax_att.plot(att["t"], att["DesRoll"], color="#9ca3af", lw=0.7, label="roll demanded")
        ax_att.plot(att["t"], att["Roll"], color="#ea580c", lw=0.8, label="roll")
        ax_att.plot(att["t"], att["Pitch"], color="#16a34a", lw=0.8, label="pitch")
        ax_att.legend(loc="upper left", fontsize=7, frameon=False, ncol=3)
    ax_att.set_ylabel("Attitude (°)")
    ax_att.set_xlabel("Time since log start (s)")

    gps = log.get("GPS")
    if gps is not None and "Status" in gps and (gps["Status"] >= 3).any():
        fix = gps["Status"] >= 3
        x, y = local_xy(gps["Lat"][fix], gps["Lng"][fix], gps["Lat"][fix][0], gps["Lng"][fix][0])
        sc = ax_trk.scatter(x, y, c=gps["t"][fix], s=2, cmap="viridis")
        ax_trk.plot(0, 0, "*", color="#ef4444", ms=10)
        ax_trk.set_aspect("equal", adjustable="datalim")
        fig.colorbar(sc, ax=ax_trk, shrink=0.8, label="time (s)")
    else:
        ax_trk.text(0.5, 0.5, "no GPS fix", ha="center", va="center", transform=ax_trk.transAxes, color="#6b7280")
    ax_trk.set_title("Track, metres from start", fontsize=9)
    ax_trk.grid(alpha=0.25)

    bat = log.get("BAT")
    if bat is not None and "Volt" in bat:
        ax_bat.plot(bat["t"], bat["Volt"], color="#1d4ed8", lw=0.9)
        ax_bat.set_ylabel("Volts", color="#1d4ed8")
        if "Curr" in bat and np.nanmax(bat["Curr"]) > 0:
            a2 = ax_bat.twinx()
            a2.plot(bat["t"], bat["Curr"], color="#f97316", lw=0.7, alpha=0.8)
            a2.set_ylabel("Amps", color="#f97316")
    ax_bat.set_title("Battery", fontsize=9)
    ax_bat.grid(alpha=0.25)

    k4 = log.get("XKF4", 0) or log.get("NKF4", 0)
    if k4 is not None:
        for f, lab in (("SV", "vel"), ("SP", "pos"), ("SH", "hgt"), ("SM", "mag")):
            if f in k4:
                ax_ekf.plot(k4["t"], k4[f], lw=0.7, label=lab)
        ax_ekf.axhline(1, color="#ef4444", lw=0.6, ls=":")
        ax_ekf.legend(fontsize=7, frameon=False, ncol=4, loc="upper left")
    ax_ekf.set_title("EKF innovation ratios", fontsize=9)
    ax_ekf.set_xlabel("Time (s)")
    ax_ekf.grid(alpha=0.25)

    n = r.counts
    colour = "#dc2626" if n["FAIL"] else "#d97706" if n["WARN"] else "#16a34a"
    fig.suptitle(f"{os.path.basename(r.path)} · {r.firmware}" + (f" · {r.board}" if r.board else ""), x=0.06, ha="left", fontsize=11, weight="bold")
    fig.text(0.06, 0.935, f"{r.verdict}   (PASS {n['PASS']} · WARN {n['WARN']} · FAIL {n['FAIL']})", color=colour, fontsize=10, weight="bold")
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return path
