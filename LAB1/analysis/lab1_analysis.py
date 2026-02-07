import os, sqlite3, glob, math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

BAGS = [
    ("occluded_5min", "Occluded (Stationary)"),
    ("open_sky_5min", "Open Sky (Stationary)"),
    ("walking_200m", "Walking (200–300m)")
]
OUT = "analysis_outputs"
os.makedirs(OUT, exist_ok=True)

def get_db3(bag_dir):
    files = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if not files:
        raise FileNotFoundError(f"No .db3 in {bag_dir}")
    return files[0]

def read_messages_as_bytes(db3_path, topic_name="/gps"):
    con = sqlite3.connect(db3_path)
    cur = con.cursor()

    # Find topic id + type
    cur.execute("SELECT id, name, type FROM topics WHERE name=?", (topic_name,))
    row = cur.fetchone()
    if row is None:
        raise RuntimeError(f"Topic {topic_name} not found in bag")
    topic_id, topic, msg_type = row

    # Fetch timestamp + raw serialized bytes
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp ASC", (topic_id,))
    data = cur.fetchall()
    con.close()
    return msg_type, data

def deserialize_gpsmsg(msg_type, rows):
    # Use ROS2 runtime to deserialize CDR into Python message instance
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    Msg = get_message(msg_type)
    t = []
    lat = []; lon = []; alt = []; hdop = []
    e = []; n = []; utc = []; zone = []; letter = []

    for ts, blob in rows:
        m = deserialize_message(blob, Msg)
        # time seconds relative from bag timestamps (ns)
        t.append(ts * 1e-9)

        # Field names might be lowercase (your actual message)
        lat.append(getattr(m, "latitude", float("nan")))
        lon.append(getattr(m, "longitude", float("nan")))
        alt.append(getattr(m, "altitude", float("nan")))
        hdop.append(getattr(m, "hdop", float("nan")))
        e.append(getattr(m, "utm_easting", float("nan")))
        n.append(getattr(m, "utm_northing", float("nan")))
        utc.append(getattr(m, "utc", float("nan")))
        zone.append(str(getattr(m, "zone", "")))
        letter.append(str(getattr(m, "letter", "")))

    df = pd.DataFrame({
        "time": np.array(t),
        "lat": np.array(lat),
        "lon": np.array(lon),
        "alt": np.array(alt),
        "hdop": np.array(hdop),
        "e": np.array(e),
        "n": np.array(n),
        "utc": np.array(utc),
        "zone": zone,
        "letter": letter
    })
    return df

def radial_error_from_first(df):
    e0 = df["e"].iloc[0]
    n0 = df["n"].iloc[0]
    return np.sqrt((df["e"]-e0)**2 + (df["n"]-n0)**2)

def line_fit_rmse(df):
    # Fit n = a*e + b and compute RMSE (meters)
    x = df["e"].to_numpy()
    y = df["n"].to_numpy()
    if len(x) < 3:
        return np.nan
    a, b = np.polyfit(x, y, 1)
    yhat = a*x + b
    rmse = float(np.sqrt(np.mean((y - yhat)**2)))
    return rmse

summary = []

for bag, label in BAGS:
    print(f"\nProcessing: {bag}")
    db3 = get_db3(bag)
    msg_type, rows = read_messages_as_bytes(db3, "/gps")
    df = deserialize_gpsmsg(msg_type, rows)

    # Make time start at 0 for plotting
    df["t0"] = df["time"] - df["time"].iloc[0]

    # Scatter N vs E (shifted by first sample)
    E = df["e"] - df["e"].iloc[0]
    N = df["n"] - df["n"].iloc[0]
    plt.figure()
    plt.scatter(E, N, s=8)
    plt.xlabel("Easting delta (m)")
    plt.ylabel("Northing delta (m)")
    plt.title(f"{label}: Northing vs Easting (shifted)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, f"{bag}_scatter.png"))
    plt.close()

    # Altitude vs time
    plt.figure()
    plt.plot(df["t0"].to_numpy(), df["alt"].to_numpy())
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title(f"{label}: Altitude vs Time")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(OUT, f"{bag}_altitude.png"))
    plt.close()

    # Error histogram (stationary bags only)
    err = radial_error_from_first(df)
    err_mean = float(np.mean(err)) if len(err) else np.nan
    err_std  = float(np.std(err)) if len(err) else np.nan

    if "Stationary" in label:
        plt.figure()
        plt.hist(err.to_numpy(), bins=30)
        plt.xlabel("Position error from first sample (m)")
        plt.ylabel("Count")
        plt.title(f"{label}: Error Histogram")
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(OUT, f"{bag}_hist.png"))
        plt.close()

    rmse_line = line_fit_rmse(df) if "Walking" in label else np.nan
    mean_hdop = float(np.nanmean(df["hdop"])) if len(df) else np.nan

    summary.append({
        "Bag": bag,
        "Label": label,
        "Msgs": int(len(df)),
        "Mean error from first (m)": err_mean,
        "Std error from first (m)": err_std,
        "Line fit RMSE (m) [walking]": rmse_line,
        "Mean HDOP": mean_hdop
    })

summary_df = pd.DataFrame(summary)
summary_path = os.path.join(OUT, "summary.csv")
summary_df.to_csv(summary_path, index=False)

print("\n✅ DONE. Outputs saved in:", OUT)
print(" - scatter plots: *_scatter.png")
print(" - altitude plots: *_altitude.png")
print(" - histograms (stationary): *_hist.png")
print(" - summary table:", summary_path)
