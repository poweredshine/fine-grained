import serial
import time

# ====== Configuration Parameters ======
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TIMEOUT = 1
COMMAND_DELAY = 0.2

# ====== AG Gripper (DH-Robotics) Modbus-RTU Commands ======
# Frame: [Slave 01][FC 06][Reg Hi][Reg Lo][Val Hi][Val Lo][CRC Lo][CRC Hi]
# CRC16-Modbus. Entries marked (M) confirmed directly from manual.
COMMANDS = {
    # --- Initialization ---
    "motor_enable":    "01 06 01 00 00 01 49 F6",  # Initialize  reg 0x0100=1   (M)

    # --- Force, reg 0x0101, valid range 20–100% ---
    "force_20":        "01 06 01 01 00 14 D9 F9",  # 20% (minimum)
    "force_25":        "01 06 01 01 00 19 18 3C",  # 25%
    "force_50":        "01 06 01 01 00 32 58 23",  # 50%
    "force_100":       "01 06 01 01 00 64 D8 1D",  # 100%

    # --- Position, reg 0x0103, 0=closed … 1000=open (unit: ‰) ---
    "clamp_min":       "01 06 01 03 00 00 78 36",  # Fully closed   (0‰)
    "pos_quarter":     "01 06 01 03 00 FA F8 75",  # Open 25%     (250‰)
    "pos_half":        "01 06 01 03 01 F4 78 21",  # Open 50%     (500‰)   (M)
    "pos_3quarter":    "01 06 01 03 02 EE F9 1A",  # Open 75%     (750‰)
    "clamp_max":       "01 06 01 03 03 E8 78 88",  # Fully open  (1000‰)

    # --- Read status (FC 03), all confirmed from manual ---
    "read_init_state": "01 03 02 00 00 01 85 B2",  # 0=not init, 1=init        (M)
    "read_grip_state": "01 03 02 01 00 01 D4 72",  # 0=moving,1=arrived,2=caught,3=dropped (M)
    "read_position":   "01 03 02 02 00 01 24 72",  # Actual position now        (M)

    # --- Configuration ---
    "save_param":      "01 06 03 00 00 01 48 4E",  # Save to flash              (M)
    "io_mode_off":     "01 06 04 02 00 00 29 3A",  # I/O mode OFF               (M)
    "io_mode_on":      "01 06 04 02 00 01 E8 FA",  # I/O mode ON
}

# ====== Menu layout ======
MENU = [
    ("0", "motor_enable",    "Initialize gripper  (wait ~3 sec)"),
    ("",  None,              "--- Force ---"),
    ("1", "force_20",        "Force 20%  (minimum)"),
    ("2", "force_25",        "Force 25%"),
    ("3", "force_50",        "Force 50%"),
    ("4", "force_100",       "Force 100%"),
    ("",  None,              "--- Posisi ---"),
    ("5", "clamp_min",       "F  (0‰)"),
    ("6", "pos_quarter",     "Open 25%   (250‰)"),
    ("7", "pos_half",        "Open 50%   (500‰)"),
    ("8", "pos_3quarter",    "Open 75%   (750‰)"),
    ("9", "clamp_max",       "Open penuh (1000‰)"),
    ("",  None,              "--- Read status ---"),
    ("r", "read_init_state", "Read status inisialisasi"),
    ("g", "read_grip_state", "Read gripper state"),
    ("p", "read_position",   "Read posisi saat ini"),
]

ser = None

def initialize_serial():
    global ser
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=TIMEOUT,
        )
        print(f"[{time.strftime('%H:%M:%S')}] Connected to {PORT}")
        if not ser.is_open:
            ser.open()
    except Exception as e:
        print(f"[{time.strftime('%H:%M:%S')}] Serial init failed: {e}")

def send_command(command_name):
    global ser
    if ser is None or not ser.is_open:
        print(f"[{time.strftime('%H:%M:%S')}] Not connected")
        return False
    command_hex = COMMANDS.get(command_name)
    if not command_hex:
        print(f"[{time.strftime('%H:%M:%S')}] Unknown command: {command_name}")
        return False
    try:
        ser.write(bytes.fromhex(command_hex))
        print(f"[{time.strftime('%H:%M:%S')}] Sent: {command_name}  [{command_hex}]")
        time.sleep(COMMAND_DELAY)
        response = ser.read(ser.in_waiting or 8)
        if response:
            print(f"[{time.strftime('%H:%M:%S')}] Response: {response.hex(' ').upper()}")
        return True
    except Exception as e:
        print(f"[{time.strftime('%H:%M:%S')}] Send failed: {e}")
        return False

# ====== Interactive Test Menu ======
if __name__ == "__main__":
    initialize_serial()
    if not (ser and ser.is_open):
        print("Gagal konek. Periksa port dan coba lagi.")
        raise SystemExit(1)

    # Build lookup: key → command_name
    key_map = {row[0]: row[1] for row in MENU if row[0]}

    print("\n=== AG Gripper Test (DH-Robotics) ===")
    for key, cmd, label in MENU:
        if key:
            print(f"  {key} : {label}")
        else:
            print(f"      {label}")
    print("  q : Exit\n")

    try:
        while True:
            choice = input("Pilih (0-9 / r g p / q): ").strip().lower()
            if choice == "q":
                break
            if choice not in key_map:
                print("Pilihan tidak valid.")
                continue

            cmd = key_map[choice]
            if choice == "0":
                print(">>> Menginisialisasi gripper, tunggu ~3 detik...")
                send_command(cmd)
                time.sleep(3)
                print(">>> Inisialisasi selesai.")
            else:
                send_command(cmd)
    finally:
        if ser and ser.is_open:
            ser.close()
        print("Serial port ditutup.")
