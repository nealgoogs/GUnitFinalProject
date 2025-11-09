import sys, termios, tty, time, select
import smbus

BUS = 1
ADDR = 0x08
bus = smbus.SMBus(BUS)

# Adjust if your firmware uses different codes
FWD       = 119
BWD       = 115
LEFT      = 97
RIGHT     = 100
STOP      = 120
SET_SPEED = 105

speed = 180            # default speed (0..255)
# Most keyboards wait ~0.5s before they start key-repeat. Keep this timeout
# comfortably above that so a single keypress doesn't spuriously latch-stop.
idle_timeout = 0.8     # seconds of silence before we assume the key was released
resend_every = 0.6     # refresh command periodically to be safe

last_speed_sent = None


def send(cmd, data=None):
    if data is None:
        data = [0, 0, 0]
    elif isinstance(data, int):
        data = [data, 0, 0]
    else:
        data = list(data)[:3] + [0] * (3 - len(data))
    bus.write_i2c_block_data(ADDR, cmd, data)
    time.sleep(0.005)


def send_speed(force=False):
    global last_speed_sent
    if force or last_speed_sent != speed:
        send(SET_SPEED, speed)
        last_speed_sent = speed

def get_key_nonblock(timeout=0.05):
    dr,_,_ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def enter_raw():
    fd = sys.stdin.fileno()
    st = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return fd, st

def exit_raw(fd, st):
    termios.tcsetattr(fd, termios.TCSADRAIN, st)

print("W/A/S/D to move, X/Space to stop, +/- to change speed, Q to quit")
print("Commands latch; robot stays idle until you send motion.")
try:
    send(STOP)
    last_speed_sent = None  # require explicit send before moving
except Exception:
    # Ignore transport errors during startup so we can still try to run.
    pass

fd, saved = enter_raw()
try:
    last_key_time = time.time()
    last_send_time = 0.0
    state = None          # 'w','a','s','d' or None
    last_state = None

    while True:
        k = get_key_nonblock(0.05)
        now = time.time()

        if k:
            k = k.lower()
            if k == 'q':
                send(STOP)
                break
            elif k in ('x', ' '):
                state = None
            elif k == '+':
                speed = min(255, speed + 10)
                send_speed(force=True)
                print(f"\rSpeed: {speed}   ", end='', flush=True)
            elif k == '-':
                speed = max(0, speed - 10)
                send_speed(force=True)
                print(f"\rSpeed: {speed}   ", end='', flush=True)
            elif k in ('w', 'a', 's', 'd'):
                state = k
            last_key_time = now

        # Stop if idle for too long
        if state is not None and (now - last_key_time) > idle_timeout:
            send(STOP)
            state = None

        # If state changed, send the new motion command once
        if state != last_state:
            if state is None:
                send(STOP)
            else:
                cmd = {'w':FWD, 'a':LEFT, 's':BWD, 'd':RIGHT}[state]
                send_speed(force=last_state is None)
                send(cmd)
            last_send_time = now
            last_state = state

        # Occasionally refresh command (helps some firmwares)
        if state is not None and (now - last_send_time) > resend_every:
            cmd = {'w':FWD, 'a':LEFT, 's':BWD, 'd':RIGHT}[state]
            send_speed()
            send(cmd)
            last_send_time = now

finally:
    exit_raw(fd, saved)
    print("\nExiting; stopped.")
    try:
        send(STOP)
    except Exception:
        pass
