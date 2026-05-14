from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface as RTDEIO
from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent
from pynput import keyboard as kb
from threading import Thread, Event, Lock
from collections import defaultdict
import numpy as np
import serial
import struct
import time

# ---------------------------------------------------------------------------
# Spacemouse 
# ---------------------------------------------------------------------------

class Spacemouse(Thread):
    def __init__(self, max_value=300, deadzone=(0,0,0,0,0,0), dtype=np.float32):
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        super().__init__()
        self.stop_event = Event()
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)
        self.button_state = defaultdict(lambda: False)
        self.tx_zup_spnav = np.array([
            [0,0,-1],
            [1,0,0],
            [0,1,0]
        ], dtype=dtype)

    def get_motion_state(self):
        me = self.motion_event
        state = np.array(me.translation + me.rotation,
                         dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state

    def get_motion_state_transformed(self):
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]
        tf_state = tf_state * SCALE_FACTOR
        return tf_state

    def is_button_pressed(self, button_id):
        return self.button_state[button_id]

    def stop(self):
        self.stop_event.set()
        self.join()

    def run(self):
        spnav_open()
        try:
            while not self.stop_event.is_set():
                event = spnav_poll_event()
                if isinstance(event, SpnavMotionEvent):
                    self.motion_event = event
                elif isinstance(event, SpnavButtonEvent):
                    self.button_state[event.bnum] = event.press
                else:
                    time.sleep(1/200)
        finally:
            spnav_close()


# ---------------------------------------------------------------------------
# KeyboardForceController — background listener
# ---------------------------------------------------------------------------

# Pemetaan tombol → (label, force %)
# Sesuaikan force % dengan objek yang sering dipegang
FORCE_LEVELS = {
    kb.KeyCode.from_char('1'): ("soft",   30),   # objek rapuh / lunak
    kb.KeyCode.from_char('2'): ("medium", 55),   # objek normal
    kb.KeyCode.from_char('3'): ("firm",   80),   # objek keras / berat
    kb.KeyCode.from_char('4'): ("max",   100),   # force maksimal
}

class KeyboardForceController:
    """
    Non-blocking keyboard listener in background thread.
    Left hand pick 1/2/3/4 anytime to change force level.
    """

    def __init__(self, initial_force: int = 55):
        self._force = initial_force
        self._label = "medium"
        self._lock  = Lock()
        self._listener = kb.Listener(on_press=self._on_press)

    def _on_press(self, key):
        if key in FORCE_LEVELS:
            label, force = FORCE_LEVELS[key]
            with self._lock:
                self._force = force
                self._label = label
            print(f"\n[Force] {label.upper()} — {force}%  "
                  f"(1=soft  2=medium  3=firm  4=max)\n")

    @property
    def force(self) -> int:
        with self._lock:
            return self._force

    @property
    def label(self) -> str:
        with self._lock:
            return self._label

    def start(self):
        self._listener.start()
        print(f"[Force] Keyboard activate. Init Force: {self._label} ({self._force}%)")
        print( "        Tombol: 1=soft(30%)  2=medium(55%)  3=firm(80%)  4=max(100%)")

    def stop(self):
        self._listener.stop()


# ---------------------------------------------------------------------------
# Parameter robot & gripper
# ---------------------------------------------------------------------------

ROBOT_HOST    = "192.168.0.2"
SCALE_FACTOR  = 0.1

GRIPPER_PORT     = "/dev/ttyUSB0"
GRIPPER_BAUDRATE = 115200
GRIPPER_TIMEOUT  = 1.0
GRIPPER_SLAVE_ID = 0x01

# Register AG-95 (DH Robotics Modbus RTU)
REG_INIT          = 0x0100
REG_FORCE         = 0x0101
REG_SPEED         = 0x0102
REG_POSITION      = 0x0103
REG_INIT_STATE    = 0x0200
REG_GRIPPER_STATE = 0x0201
REG_ACTUAL_POS    = 0x0202

DEFAULT_SPEED = 50
POS_OPEN      = 0
POS_CLOSED    = 1000
POS_STEP      = 100


# ---------------------------------------------------------------------------
# Modbus RTU helpers
# ---------------------------------------------------------------------------

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def _build_write_register(slave_id: int, register: int, value: int) -> bytes:
    payload = struct.pack(">BBHH", slave_id, 0x06, register, value)
    return payload + struct.pack("<H", _crc16(payload))

def _build_read_register(slave_id: int, register: int, count: int = 1) -> bytes:
    payload = struct.pack(">BBHH", slave_id, 0x03, register, count)
    return payload + struct.pack("<H", _crc16(payload))

def _parse_read_response(response: bytes, count: int = 1) -> list[int]:
    expected = 3 + count * 2 + 2
    if len(response) < expected:
        raise ValueError(f"Shorth Response: {len(response)} bytes")
    recv_crc = struct.unpack_from("<H", response, expected - 2)[0]
    if recv_crc != _crc16(response[:expected - 2]):
        raise ValueError("CRC mismatch")
    return [struct.unpack_from(">H", response, 3 + i * 2)[0] for i in range(count)]


# ---------------------------------------------------------------------------
# GripperController — AG-95 Modbus RTU
# ---------------------------------------------------------------------------

class GripperController:
     def __init__(self, port, baudrate, timeout, slave_id):
        self._slave_id   = slave_id
        self._lock       = Lock()
        self._worker     = None
        self._target_pos = POS_OPEN

        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout,
        )
        if not self.ser.is_open:
            self.ser.open()
        print(f"AG-95 connected on {port} @ {baudrate} baud")
        self._write_register(REG_SPEED, DEFAULT_SPEED)

    def _write_register(self, register: int, value: int):
        frame = _build_write_register(self._slave_id, register, value)
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            time.sleep(0.02)
            self.ser.read(8)

    def _read_register(self, register: int, count: int = 1):
        frame = _build_read_register(self._slave_id, register, count)
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            time.sleep(0.02)
            response = self.ser.read(3 + count * 2 + 2)
        try:
            return _parse_read_response(response, count)
        except ValueError as e:
            print(f"[Gripper] Read error: {e}")
            return None

    def initialize(self):
        print("Init AG-95...")
        self._write_register(REG_INIT, 0x01)
        for _ in range(50):
            time.sleep(0.1)
            result = self._read_register(REG_INIT_STATE)
            if result is not None and result[0] == 0:
                print("Init done.")
                return
        print("Warning: init timeout.")

    def set_position(self, position: int, force: int):
        position = max(POS_OPEN, min(POS_CLOSED, position))
        force    = max(20, min(100, force))
        self._target_pos = position

        if self._worker is not None and self._worker.is_alive():
            return

        def _do():
            self._write_register(REG_FORCE, force)
            self._write_register(REG_POSITION, position)

        self._worker = Thread(target=_do, daemon=True)
        self._worker.start()

    def step_close(self, force: int):
        new_pos = min(POS_CLOSED, self._target_pos + POS_STEP)
        print(f"[Gripper] close → pos={new_pos}  force={force}%")
        self.set_position(new_pos, force)

    def step_open(self, force: int):
        new_pos = max(POS_OPEN, self._target_pos - POS_STEP)
        print(f"[Gripper] open  → pos={new_pos}  force={force}%")
        self.set_position(new_pos, force)

    def get_actual_position(self) -> int | None:
        result = self._read_register(REG_ACTUAL_POS)
        return result[0] if result else None

    def get_gripper_state(self) -> str:
        state_map = {0: "moving", 1: "reached", 2: "gripped", 3: "dropped"}
        result = self._read_register(REG_GRIPPER_STATE)
        return state_map.get(result[0], "unknown") if result else "unknown"

    def close_port(self):
        if self.ser.is_open:
            self.ser.close()
        print("Gripper serial port closed.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    sm = Spacemouse(deadzone=0.2)
    sm.start()

    force_ctrl = KeyboardForceController(initial_force=55)
    force_ctrl.start()

    rtde_c = RTDEControlInterface(ROBOT_HOST)
    rtde_r = RTDEReceiveInterface(ROBOT_HOST)
    rtde_io = RTDEIO(ROBOT_HOST)

    gripper = GripperController(
        port=GRIPPER_PORT,
        baudrate=GRIPPER_BAUDRATE,
        timeout=GRIPPER_TIMEOUT,
        slave_id=GRIPPER_SLAVE_ID,
    )
    gripper.initialize()

    prev_btn = [False, False]

    try:
        while True:
            if rtde_r.getRobotMode() == 7:
                motion_state = sm.get_motion_state_transformed()
                rtde_c.speedL(motion_state, acceleration=0.5, time=0.1)

                # Force selalu dibaca fresh dari keyboard controller
                current_force = force_ctrl.force

                # Btn kiri — tutup satu step dengan force saat ini
                btn0 = sm.is_button_pressed(0)
                if btn0 and not prev_btn[0]:
                    gripper.step_close(force=current_force)
                prev_btn[0] = btn0

                # Btn kanan — buka satu step dengan force saat ini
                btn1 = sm.is_button_pressed(1)
                if btn1 and not prev_btn[1]:
                    gripper.step_open(force=current_force)
                prev_btn[1] = btn1

                time.sleep(1/100)

            else:
                print("Robot not ready.")
                time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped...")
        rtde_c.stopScript()
        sm.stop()
        force_ctrl.stop()
        gripper.close_port()


if __name__ == "__main__":
    main()