# IMU TX - ESP32 Firmware

## What This Is

Hand controller firmware for direct manual flight. Reads MPU6050 IMU and joystick, transmits to drone RX via ESP-NOW.

**No computer needed for flight** - this is a standalone manual controller that talks directly to the drone.

## Hardware

### Components

1. **ESP32-C3 Dev Board**
2. **MPU6050 6-DOF IMU** (I2C, ~$2-5)
3. **Button** (optional)
4. **Potentiometer** (optional, for throttle)
5. **LED** (status indicator)
6. **Battery** (LiPo 1S or power bank)
7. **Mounting** (wrist strap or glove)

### Wiring

```
ESP32-C3    MPU6050
--------    -------
GPIO 8  →   SDA
GPIO 9  →   SCL
3.3V    →   VCC
GND     →   GND

ESP32-C3    Joystick Module
--------    ---------------
GPIO 4  →   VRx (X-axis, future use)
GPIO 5  →   VRy (Y-axis, throttle)
GPIO 3  →   SW (button, arming)
3.3V    →   VCC
GND     →   GND

ESP32-C3    Other
--------    -----
GPIO 10 →   LED + 220Ω resistor → GND
```

### Mounting Options

**Option 1: Handheld Grip**
- 3D printed handle
- IMU on top surface
- Joystick accessible by thumb
- Battery in handle

**Option 2: Glove/Wrist Mount**
- IMU on back of hand
- Joystick on palm side (thumb control)
- Battery on wrist

**Option 3: Single-Hand Controller**
- All components in one enclosure
- Hold like game controller
- Tilt controller = tilt drone

**IMU Orientation**:
- X-axis along fingers (forward)
- Y-axis across palm (right)
- Z-axis perpendicular to palm (up)

## Software Setup

### Flash Firmware

```bash
cd IMU_TX

# Build and upload
pio run -t upload

# Monitor
pio device monitor
```

### Configuration

Edit `src/main.cpp` before building:

```cpp
// WiFi - computer hotspot
const char* WIFI_SSID = "UAV_CONTROL";
const char* WIFI_PASSWORD = "uav12345";

// Transmit rate
#define IMU_SEND_FREQUENCY_HZ 50

// Pins (adjust for your board)
#define IMU_SDA_PIN 8          // MPU6050 SDA
#define IMU_SCL_PIN 9          // MPU6050 SCL
#define JOYSTICK_X_PIN 4       // Future use
#define JOYSTICK_Y_PIN 5       // Throttle
#define JOYSTICK_BTN_PIN 3     // Arming
#define LED_PIN 10
```

## Usage

### 1. Power On

Connect battery or USB. LED will blink during calibration.

### 2. Calibrate IMU

Keep hand still for 3 seconds after power on. MPU6050 will zero itself. LED turns solid when ready.

### 3. Connect to Computer

The ESP32 connects to the computer's WiFi hotspot and broadcasts IMU data via ESP-NOW.

Computer receives via `imu_protocol_receiver` ROS node.

### 4. Control Drone

- **Tilt** controller → Drone tilts (orientation mapped 1:1)
- **Joystick Y-axis** → Throttle (up = more thrust)
- **Joystick button** → Arm/Disarm (press to arm)

## Protocol Details

### Packet Format

Sends `PKT_IMU_DATA` (type 0x12) packets:

```c
struct ImuDataPayload {
  float qw, qx, qy, qz;  // Quaternion orientation
  float throttle;        // 0.0 to 1.0
  uint32_t timestamp;    // Milliseconds
  uint8_t flags;         // Bit 0: calibrated, Bit 1: button
};
```

Total: 25 bytes payload

### Transmission

- Rate: 50 Hz (configurable)
- Transport: ESP-NOW broadcast
- Target: Computer running ROS 2
- Failsafe: Heartbeat every 500ms

## Computer Setup

The computer needs:

1. **WiFi Hotspot** - Create hotspot named "UAV_CONTROL"
2. **ROS 2 Node** - `imu_protocol_receiver` to receive packets
3. **IMU Controller** - Processes gestures
4. **Rest of stack** - PID, safety, CRSF adapter

### Create WiFi Hotspot (Ubuntu)

```bash
# Using NetworkManager
nmcli dev wifi hotspot ssid UAV_CONTROL password uav12345

# Or use GUI settings → WiFi → Create Hotspot
```

### Launch ROS Stack

```bash
cd UAV-Controller
source ros2_ws/install/setup.bash

# For simulation
./scripts/run_imu_manual_sim.sh

# For hardware (after TX_RX integration)
./scripts/run_imu_manual_hw.sh transport:=udp
```

## Troubleshooting

**LED blinking**
- Wait 3-5 seconds for init
- Check I2C wiring (GPIO 8/9)
- Verify MPU6050 address is 0x68
- Try power cycling

**No data on computer**
- Check WiFi connection
- Verify computer hotspot is "UAV_CONTROL"
- Check ESP32 serial output for IP address
- Verify ROS `imu_protocol_receiver` is running

**Wrong orientation mapping**
- Check IMU mounting orientation
- Adjust in ROS config: `imu_controller_params.yaml`
- Set `roll_scale: -1.0` to reverse axis

**Delayed response**
- Increase transmit rate: `IMU_SEND_FREQUENCY_HZ 100`
- Check WiFi signal strength
- Reduce computer processing load

## Development

### Serial Monitor

```bash
pio device monitor

# Should show:
# - IMU calibration status
# - Quaternion values
# - Throttle reading
# - Packets sent
# - Link status
```

### Testing IMU TX Alone

```bash
# Power on IMU TX
pio device monitor

# Should see:
# - MPU6050 detected
# - Roll/Pitch angles
# - Packets sent
```

### Testing Protocol

```bash
# On computer, listen for packets
# (requires custom UDP listener or ROS node)

source ros2_ws/install/setup.bash
ros2 run manual_imu_controller imu_protocol_receiver

# In another terminal, check if data arrives
ros2 topic echo /imu/data
ros2 topic echo /manual/throttle
```

## Safety Notes

- Secure mounting - don't want IMU falling off mid-flight
- Calibrate before every session
- Check battery level
- Test range - stay within WiFi coverage
- Have manual RC override ready

## Data Flow

**Direct Manual Flight** (normal operation):
```
IMU TX (hand) → ESP-NOW (PKT_IMU_DATA) → RX (drone) → CRSF → Flight Controller
```

No computer needed!

**Optional: Test with ROS Simulation**:
```
IMU TX → USB ESP32 dongle → Computer ROS → Simulator → RViz
```

Only for testing gestures before flight.

## Integration with TX_RX

The RX firmware needs one update to handle `PKT_IMU_DATA` packets. See `../TX_RX/RX_IMU_SUPPORT.md` for the code to add.

After RX update, the system supports three transmitter types:
1. Bench Test TX (auto testing)
2. Command TX (computer/ROS autonomous control)  
3. IMU TX (manual gesture control) ← This one

All use same RX, same protocol!

---

**Quick Start:**
1. Wire MPU6050 + joystick to ESP32
2. Flash this firmware: `pio run -t upload`
3. Update RX firmware (add IMU packet handler)
4. Power on, tilt to fly!

