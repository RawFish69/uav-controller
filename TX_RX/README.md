# Custom TX/RX Protocol - Commands

## Build

```bash
# Build TX
pio run -e tx

# Build RX
pio run -e rx

# Build all
pio run

# Clean build
pio run -t clean
```

## Upload

```bash
# Upload TX
pio run -e tx -t upload

# Upload RX
pio run -e rx -t upload

# Upload to specific port
pio run -e tx -t upload --upload-port COM3
```

## Serial Monitor

```bash
# Start monitor
pio device monitor

# Monitor specific port
pio device monitor -p COM3

# List available ports
pio device list
```

## Common Workflows

```bash
# Build, upload, and monitor TX
pio run -e tx -t upload && pio device monitor

# Build, upload, and monitor RX
pio run -e rx -t upload && pio device monitor

# Clean and rebuild
pio run -t clean && pio run -e tx
```

## Configuration

Edit `src/main.cpp`:

### TX Side:
```cpp
// Change frequency (50, 100, 150, 250, 500 Hz)
#define RC_SEND_FREQUENCY_HZ 100

// Enable/disable bench test mode
#define BENCH_TEST_MODE true
```

### RX Side:
```cpp
// Enable CRSF output to flight controller
#define ENABLE_CRSF_OUTPUT true   // true = production mode (CRSF enabled, no debug)
                                   // false = debug mode (CRSF disabled, full serial output)

// CRSF frequency (should match TX)
#define CRSF_OUTPUT_FREQUENCY_HZ 100
```

**Note for M5Stack C3 boards:** Serial1 conflicts with USB Serial. Use `ENABLE_CRSF_OUTPUT = false` for debugging, `true` for flying (no serial monitor).
Should be fine for S3 boards...