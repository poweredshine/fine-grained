# UR3 Gripper Control — DH-Robotics AG Series (Modbus-RTU over RS485)

This repository contains Python scripts for controlling a **DH-Robotics AG-series** parallel gripper integrated with a Universal Robots UR3 (CB3). Communication is **Modbus-RTU** over RS485 via a USB-to-RS485 adapter.

## 🛠 Hardware Setup

### 1. Requirements
* **Robot:** UR3 (CB3 series)
* **Gripper:** DH-Robotics AG series (e.g., AG-95 / AG-105) — RS485, Modbus-RTU
* **Communication:** USB-to-RS485 adapter (typically appears as `/dev/ttyUSB0`)
* **Power:** External 24V DC power supply
* **Host:** Ubuntu 22.04/24.04

### 2. Wiring
* **RS485 A (+):** Gripper A+ → Adapter A+
* **RS485 B (−):** Gripper B− → Adapter B−
* **VCC (+24V):** Gripper power → External +24V
* **GND:** Common ground between supply and gripper

### 3. Serial Parameters
| Parameter | Value     |
|-----------|-----------|
| Baudrate  | 115200    |
| Data bits | 8         |
| Parity    | None      |
| Stop bits | 1         |
| Slave ID  | 0x01      |

---
## 🚀 Software Setup

1. Install the required Python library:
```bash
pip install pyserial
```

2. **Port Permissions** — Ubuntu restricts access to serial ports by default. Grant permissions to your device (usually `/dev/ttyUSB0`):
```bash
sudo chmod 666 /dev/ttyUSB0
```

3. **Run the Interactive Test Menu:**
```bash
python3 gripper_test.py
```

---
## 📋 Interactive Menu

The script opens an interactive prompt that sends pre-built Modbus-RTU frames to the AG gripper. Each option maps to a single register write (FC 06) or status read (FC 03).

| Key | Action              | Description                          |
|-----|---------------------|--------------------------------------|
| `0` | Initialize          | Enable motor (reg `0x0100 = 1`). **Wait ~3 s** for the gripper to home. |
| `1` | Force 20%           | Minimum grip force (reg `0x0101`)    |
| `2` | Force 25%           | Set grip force to 25%                |
| `3` | Force 50%           | Set grip force to 50%                |
| `4` | Force 100%          | Maximum grip force                   |
| `5` | Fully closed        | Position 0‰ (reg `0x0103`)           |
| `6` | Open 25%            | Position 250‰                        |
| `7` | Open 50%            | Position 500‰                        |
| `8` | Open 75%            | Position 750‰                        |
| `9` | Fully open          | Position 1000‰                       |
| `r` | Read init state     | `0` = not initialized, `1` = ready   |
| `g` | Read grip state     | `0` moving, `1` arrived, `2` caught, `3` dropped |
| `p` | Read position       | Current position (‰)                 |
| `q` | Exit                | Close the serial port and quit       |

> **Important:** Always run `0` (Initialize) once after power-on before issuing position or force commands.

---
## 🧩 Modbus-RTU Frame Reference

Frame layout used by the AG series:
```
[Slave 01][FC 06/03][Reg Hi][Reg Lo][Val Hi][Val Lo][CRC Lo][CRC Hi]
```
CRC is standard CRC16-Modbus.

Key registers:
| Register | R/W | Range      | Meaning                          |
|----------|-----|------------|----------------------------------|
| `0x0100` | W   | 1          | Initialize / enable motor        |
| `0x0101` | W   | 20 – 100   | Grip force (%)                   |
| `0x0103` | W   | 0 – 1000   | Target position (‰: 0 closed, 1000 open) |
| `0x0200` | R   | 0 / 1      | Initialization state             |
| `0x0201` | R   | 0 – 3      | Gripper state (moving/arrived/caught/dropped) |
| `0x0202` | R   | 0 – 1000   | Actual position                  |
| `0x0300` | W   | 1          | Save parameters to flash         |
| `0x0402` | W   | 0 / 1      | I/O mode off / on                |

---
## 🗂 Code Architecture

[gripper_test.py](gripper_test.py) contains:

* **Configuration** (`PORT`, `BAUDRATE`, `TIMEOUT`, `COMMAND_DELAY`) — edit at the top of the file to match your hardware.
* **`COMMANDS` dict** — pre-computed Modbus-RTU hex frames (with valid CRC) for every menu action.
* **`MENU` list** — declarative table that drives the interactive prompt.
* **`initialize_serial()`** — opens the serial port and stores the handle in the global `ser`.
* **`send_command(name)`** — looks up a frame, writes it, then reads back the gripper response and prints it as hex.

---
## 🛠 Troubleshooting

* **`Serial init failed` / permission denied** → re-run `sudo chmod 666 /dev/ttyUSB0`, or add your user to the `dialout` group.
* **No response from gripper** → check A/B polarity, 24 V supply, common ground, and that the slave ID matches `0x01`.
* **Gripper does not move after a command** → make sure you sent `0` (Initialize) and waited ~3 s for homing.
