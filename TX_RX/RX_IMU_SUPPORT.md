# RX Firmware Update for IMU TX Support

## What Needs to Change

The RX currently handles:
- `PKT_RC_COMMAND` - RC channels (from bench test TX)
- `PKT_DIRECT_CMD` - Direct commands (from computer/autonomous mode)

**Add support for**: `PKT_IMU_DATA` - Quaternion orientation from IMU TX

## Code Changes Needed in TX_RX/src/main.cpp (RX section)

### 1. Add Packet Handler

In the `CustomProtocol_ParsePacket` callback or wherever packets are processed:

```cpp
#ifdef BUILD_RX

// Add this helper function
void quaternionToEuler(float qw, float qx, float qy, float qz, 
                      float* roll, float* pitch, float* yaw) {
  // Roll (x-axis rotation)
  float sinr_cosp = 2.0 * (qw * qx + qy * qz);
  float cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  *roll = atan2(sinr_cosp, cosr_cosp);
  
  // Pitch (y-axis rotation)
  float sinp = 2.0 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    *pitch = copysign(M_PI / 2, sinp);
  else
    *pitch = asin(sinp);
  
  // Yaw (z-axis rotation)
  float siny_cosp = 2.0 * (qw * qz + qx * qy);
  float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  *yaw = atan2(siny_cosp, cosy_cosp);
}

float eulerToCRSF(float angle_rad, float max_angle = 0.5) {
  // Map angle (radians) to CRSF range
  // max_angle = ±30 degrees = ±0.524 rad
  float normalized = angle_rad / max_angle;  // -1..1
  normalized = constrain(normalized, -1.0, 1.0);
  return 992 + (normalized * 400);  // 992 ± 400
}

float throttleToCRSF(float throttle) {
  // Map 0..1 to CRSF range 172..1811
  return 172 + (throttle * (1811 - 172));
}

// In packet parsing callback, add case for IMU_DATA:
void onPacketReceived(uint8_t packetType, uint8_t* payload, uint16_t len) {
  if (packetType == PKT_IMU_DATA && len >= sizeof(ImuDataPayload)) {
    ImuDataPayload* imu = (ImuDataPayload*)payload;
    
    // Convert quaternion to Euler
    float roll, pitch, yaw;
    quaternionToEuler(imu->qw, imu->qx, imu->qy, imu->qz, &roll, &pitch, &yaw);
    
    // Convert to CRSF channels
    uint16_t channels[16];
    channels[0] = eulerToCRSF(roll);           // Roll
    channels[1] = eulerToCRSF(pitch);          // Pitch  
    channels[2] = throttleToCRSF(imu->throttle); // Throttle
    channels[3] = eulerToCRSF(yaw);            // Yaw
    channels[4] = (imu->flags & 0x02) ? 1811 : 172;  // AUX1 (armed)
    
    // Rest centered
    for (int i = 5; i < 16; i++) {
      channels[i] = 992;
    }
    
    // Send to FC via CRSF
    crsfBridge.sendRcChannels(channels);
  }
}

#endif // BUILD_RX
```

### 2. Update protocol.h

Already has `PKT_IMU_DATA = 0x12` and `ImuDataPayload` struct defined.

No changes needed - protocol header is shared between IMU TX and RX!

## Testing

### 1. Flash Updated RX

```bash
cd TX_RX
# Add the code above to src/main.cpp
pio run -e receiver -t upload
```

### 2. Test with IMU TX

```bash
# Power on IMU TX (hand controller)
# Power on RX (on drone, props OFF!)
# Move IMU TX, check RX serial output
# Should see RC channels changing
```

### 3. Verify on Flight Controller

Connect RX to FC via CRSF (GPIO 21 → FC RX pin).
Check FC configurator - should see RC inputs moving when you tilt IMU TX.

## Result

After this update, the system supports three transmitter types:
1. **Bench Test TX** - Auto bench testing (PKT_RC_COMMAND)
2. **Command TX** - Computer/ROS control (PKT_DIRECT_CMD)
3. **IMU TX** - Manual gesture control (PKT_IMU_DATA)

All use the same RX and same CustomProtocol!

---

**Note**: The ROS IMU controller package is only for simulation/testing, not actual flight.

