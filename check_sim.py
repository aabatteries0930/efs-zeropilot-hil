import math
import socket
import struct
import threading
import time
from collections import deque
from telemetry_receiver import open_telemetry_link, next_packet
from hil_config import CHECK_FORMAT, CHECK_PORT, CHECK_TOLERANCE as TOL, UDP_HOST
from hil_config import TM_GPS, TM_BATTERY, TM_SERVO, TM_HEARTBEAT, CHECK_TYPES, PRINT_TYPES
from hil_config import LAG_MIN_S, LAG_MAX_S, LAG_SEARCH_S
from hil_config import ESP32_VOLT, ESP32_CURRENT, HB_MIN_S, HB_MAX_S, PWM_CHANNEL_NAMES
from hil_config import EXPECTED_MAV_TYPE, EXPECTED_AUTOPILOT
from hil_main import pwm_us_to_normalized
sim_buf = deque(maxlen=2000)   # ring buffer of (time, state)
lock = threading.Lock()
last_heartbeat = {}

# conversion constants
M_PER_DEG_LAT = 111_320.0
DEGREES_FIX = 1e7
MILLI = 1000.0
CENTI = 100.0
M_PER_FT = 0.3048
UNKNOWN = 65535 #MAVLink's "no data" value

def sim_listener():
    """Thread: receives sim packets from flight_server.py and stores them in SI units."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_HOST, CHECK_PORT))
    while True:
        data, _ = sock.recvfrom(1024)
        t, alt_ft, vc_ft, lat, lon, psi, *ctrl = struct.unpack(CHECK_FORMAT, data)
        with lock:
            sim_buf.append((t, {"alt": alt_ft * M_PER_FT, #ft to m
                                "vel": vc_ft * M_PER_FT, #ft/s to m/s
                                "lat": lat, "lon": lon, #degrees
                                "hdg": psi, #degrees (yaw)
                                "ctrl": ctrl})) #normalized controls

def snapshot():
    with lock:
        return list(sim_buf)

def ang_diff(a, b):
    return abs((a - b + 180) % 360 - 180)

def display(name, telemetry, sim, diff, tol, unit):
    tag = "PASS" if diff <= tol else "FAIL"
    print(f"[{tag}] {name}: telemetry={telemetry:.3f} sim={sim:.3f} diff={diff:.3f}{unit} (tol {tol}{unit})")

def gps_rows(tm, s):
    m_lon = M_PER_DEG_LAT * math.cos(math.radians(tm["lat"]))     # metres per degree of longitude
    rows = [("lat", tm["lat"], s["lat"], abs(tm["lat"] - s["lat"]) * M_PER_DEG_LAT, TOL["lat_m"], "m"),
            ("lon", tm["lon"], s["lon"], abs(tm["lon"] - s["lon"]) * m_lon, TOL["lon_m"], "m"),
            ("alt", tm["alt"], s["alt"], abs(tm["alt"] - s["alt"]), TOL["alt_m"], "m")]
    if tm["vel"] is not None:
        rows.append(("vel", tm["vel"], s["vel"], abs(tm["vel"] - s["vel"]), TOL["vel_ms"], "m/s"))
        if tm["vel"] > 2 and tm["cog"] is not None:              # course only means something while moving
            rows.append(("cog", tm["cog"], s["hdg"], ang_diff(tm["cog"], s["hdg"]), TOL["cog_deg"], "deg"))
    return rows

def servo_rows(norms, s):
    return [(f"servo {name}", norms[i], s["ctrl"][i], abs(norms[i] - s["ctrl"][i]), TOL["servo_norm"], "")
            for i, name in enumerate(PWM_CHANNEL_NAMES)]

def all_pass(rows):
    return all(diff <= tol for _, _, _, diff, tol, _ in rows)

def find_matches(samples, t_rx, lookback_s, rows_fn, tm):
    window = [(t_rx - t, s) for t, s in samples if t_rx - t <= lookback_s]
    matches = [(lag, s) for lag, s in window if all_pass(rows_fn(tm, s))]
    return window, matches

def nearest(window, lag_target):
    return min(window, key=lambda m: abs(m[0] - lag_target))

def check(pkt, t_rx):
    kind, field = pkt["type"], pkt["fields"]

    if kind == TM_GPS:
        # convert telemetry units to SI
        tm = {"lat": field["lat"] / DEGREES_FIX, #deg x 1e7 -> deg
              "lon": field["lon"] / DEGREES_FIX,
              "alt": field["alt"] / MILLI, #mm -> m
              "vel": field["vel"] / CENTI if field["vel"] != UNKNOWN else None, #cm/s -> m/s
              "cog": field["cog"] / CENTI if field["cog"] != UNKNOWN else None} #cdeg -> deg

        window, matches = find_matches(snapshot(), t_rx, LAG_SEARCH_S, gps_rows, tm)
        if not window:
            print("[NO DATA] GPS_RAW_INT: sim buffer is empty"); return

        in_range = [m for m in matches if LAG_MIN_S <= m[0] <= LAG_MAX_S]
        if in_range:
            lag, s = in_range[len(in_range) // 2]
        elif matches:                      
            lag, s = matches[len(matches) // 2]
        else:
            lag, s = nearest(window, (LAG_MIN_S + LAG_MAX_S) / 2)

        print(f"[GPS] compared against sim sample from {lag:.3f}s ago")
        for row in gps_rows(tm, s):
            display(*row)
        if in_range:
            print(f"[PASS] lag: telemetry matches the sim within the expected {LAG_MIN_S}-{LAG_MAX_S}s delay")
        elif matches:
            print(f"[FAIL] telemetry only matches the sim from {lag:.2f}s ago (expected {LAG_MIN_S}-{LAG_MAX_S}s)")

    elif kind == TM_BATTERY:
        mv, ca = field["voltages"][0], field["current_battery"]
        display("batt V", mv, ESP32_VOLT * MILLI, abs(mv - ESP32_VOLT * MILLI), TOL["batt_mv"], "mV")
        display("batt I", ca, ESP32_CURRENT * CENTI, abs(ca - ESP32_CURRENT * CENTI), TOL["batt_ca"], "cA")

    elif kind == TM_SERVO:
        norms = []
        for i in range(len(PWM_CHANNEL_NAMES)):     
            us = field[f"servo{i + 1}_raw"]
            norms.append(pwm_us_to_normalized(us, bipolar=(i < 3)))

        window, matches = find_matches(snapshot(), t_rx, LAG_MAX_S, servo_rows, norms)
        if not window:
            print("[NO DATA] SERVO_OUTPUT_RAW: sim buffer is empty"); return
        lag, s = matches[len(matches) // 2] if matches else nearest(window, LAG_MAX_S / 2)
        print(f"[SERVO] compared against sim sample from {lag:.3f}s ago")
        for row in servo_rows(norms, s):
            display(*row)

    elif kind == TM_HEARTBEAT:
        ok = field["type"] == EXPECTED_MAV_TYPE and field["autopilot"] == EXPECTED_AUTOPILOT
        print(f"[{'PASS' if ok else 'FAIL'}] heartbeat identity: type={field['type']} (expected {EXPECTED_MAV_TYPE}), "
              f"autopilot={field['autopilot']} (expected {EXPECTED_AUTOPILOT})")
        if "t" in last_heartbeat:
            dif_time = t_rx - last_heartbeat["t"]
            tag = "PASS" if HB_MIN_S <= dif_time <= HB_MAX_S else "FAIL"
            print(f"[{tag}] heartbeat gap {dif_time:.2f}s (expected {HB_MIN_S}-{HB_MAX_S}s)")
        last_heartbeat["t"] = t_rx

def handle(pkt, t_rx):
    if pkt["type"] in CHECK_TYPES:
        check(pkt, t_rx)
    elif pkt["type"] in PRINT_TYPES:
        print(f"[PRINT] {pkt['type']} at {t_rx:.2f}s: {pkt['fields']}")

def main():
    threading.Thread(target=sim_listener, daemon=True).start()
    conn = open_telemetry_link()
    while True:
        pkt = next_packet(conn)
        if pkt is not None:
            try:
                handle(pkt, time.monotonic())
            except Exception as e:
                print(f"[ERROR] {e} while handling {pkt['type']}")

if __name__ == "__main__":
    main()