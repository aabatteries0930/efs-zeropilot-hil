from pymavlink import mavutil

SERIAL_PORT = "/dev/ttyUSB0"
BAUD = 57600

# takes in data message packages into dict format (msg type, value, timestamp in ms since boot)
def to_packet(msg):
    msg_type = msg.get_type()
    fields = msg.to_dict()

    if "time_boot_ms" in fields:
        timestamp_ms = fields["time_boot_ms"]
    elif "time_usec" in fields:
        timestamp_ms = fields["time_usec"] // 1000
    else:
        timestamp_ms = None

    packet = {
        "type": msg_type,
        "fields": fields,
        "time_boot_ms": timestamp_ms,
    }
    return packet

def open_telemetry_link(port = SERIAL_PORT, baud = BAUD):
    return mavutil.mavlink_connection(port, baud = baud)

def next_packet(conn):
    msg = conn.recv_match(blocking = True)
    if msg is None:
        return None
    return to_packet(msg)
