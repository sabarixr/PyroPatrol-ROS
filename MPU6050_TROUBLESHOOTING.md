# MPU6050 I2C Communication Issues - Troubleshooting Guide

## Problem Summary

Your I2C scan shows the MPU6050 is detected at address `0x68`, but when running the robot system, the IMU node fails to read data properly. This is a **common I2C bus contention and timing issue**.

## Root Causes

### 1. **High CPU Usage** (Most Likely Cause)
- Your `mpu6050_node` process was using **36.9% CPU**
- This indicates the I2C reads are failing and the node is constantly retrying
- Normal IMU nodes should use < 5% CPU

### 2. **I2C Bus Timing Issues**
- Reading at 100Hz (every 10ms) is aggressive for I2C
- When motors are running, electrical noise can interfere
- No error recovery means one failed read causes cascading failures

### 3. **Lack of Retry Logic**
- Original code had no retry mechanism for I2C failures
- A single glitch would cause data loss

### 4. **Insufficient Initialization Delays**
- MPU6050 needs time to stabilize after configuration changes
- Original code had minimal delays

## Solution Applied

The `fix_mpu6050_issue.sh` script implements these fixes:

### 1. **Robust Error Handling**
```python
def read_byte_with_retry(self, reg):
    """Read with automatic retries"""
    for attempt in range(self.max_retries):
        try:
            return self.bus.read_byte_data(self.addr, reg)
        except Exception as e:
            if attempt == self.max_retries - 1:
                raise e
            time.sleep(0.005)  # 5ms delay before retry
```

### 2. **Automatic Reinitialization**
```python
if self.read_failure_count >= self.max_consecutive_failures:
    self.get_logger().error('Too many failures, reinitializing...')
    self.init_sensor()
```

### 3. **Reduced Polling Rate**
- Changed from **100Hz (10ms)** to **50Hz (20ms)**
- Reduces I2C bus congestion
- Still more than sufficient for robot control

### 4. **Proper Initialization**
```python
# Reset device first
self.write_byte_with_retry(0x6B, 0x80)
time.sleep(0.1)  # Wait for reset

# Wake up with delays between steps
self.write_byte_with_retry(0x6B, 0x00)
time.sleep(0.05)

# Configure with delays
self.write_byte_with_retry(0x1C, 0x00)
time.sleep(0.01)
```

## Quick Fix

Run the automated fix script:

```bash
cd ~/frr_ws
./fix_mpu6050_issue.sh
```

This will:
1. ✓ Stop running processes
2. ✓ Backup original node
3. ✓ Install robust version
4. ✓ Check I2C configuration
5. ✓ Rebuild package
6. ✓ Test sensor

## Manual Verification

### 1. Test MPU6050 Directly
```bash
cd ~/frr_ws
python3 diagnose_mpu6050.py
```

Should show:
- ✓ I2C bus opened
- ✓ MPU6050 detected
- ✓ 10/10 samples successful

### 2. Check Process CPU Usage
```bash
# Start the robot
cd ~/frr_ws
source install/setup.bash
ros2 launch frr_bringup rover_bringup.launch.py

# In another terminal
ps aux | grep mpu6050
```

CPU usage should be **< 10%** (was 36.9% before)

### 3. Monitor IMU Data
```bash
# Check publish rate
ros2 topic hz /imu/mpu6050

# Should show ~50 Hz

# View actual data
ros2 topic echo /imu/mpu6050
```

## Additional Optimizations (Optional)

### Reduce I2C Bus Speed

If you still have issues, slow down the I2C bus:

```bash
# Edit config file
sudo nano /boot/firmware/config.txt
# or
sudo nano /boot/config.txt

# Add or change this line:
dtparam=i2c_arm_baudrate=100000

# Save and reboot
sudo reboot
```

**Speed recommendations:**
- `400000` (400kHz) - Standard, usually works
- `100000` (100kHz) - More reliable, recommended if issues persist
- `50000` (50kHz) - Very stable but slower

### Check for I2C Bus Contention

Make sure only the ROS node accesses the MPU6050:

```bash
# Check for other processes
ps aux | grep -E "python|mpu|imu" | grep -v grep

# Should only see ros2 processes
```

### Add Hardware Pull-up Resistors

If software fixes don't fully resolve the issue:
- Add 4.7kΩ resistors between SDA → 3.3V
- Add 4.7kΩ resistors between SCL → 3.3V
- This improves signal integrity

### Check Power Supply

MPU6050 needs clean 3.3V power:
- Measure voltage at VCC pin (should be 3.2-3.4V)
- Add 0.1µF ceramic capacitor near VCC/GND if possible
- Ensure good ground connection

## Monitoring Tools

### CPU Usage
```bash
htop
# Press F4, type "mpu", Enter to filter
```

### I2C Bus Activity
```bash
# Install i2c-tools if needed
sudo apt install i2c-tools

# Monitor bus
watch -n 0.5 'sudo i2cdetect -y 1'
```

### ROS Topic Statistics
```bash
# Message rate
ros2 topic hz /imu/mpu6050

# Bandwidth
ros2 topic bw /imu/mpu6050

# Latency
ros2 topic delay /imu/mpu6050
```

## Understanding the Fix

### Before (Original Code)
```python
# Timer at 100Hz - very fast
self.timer = self.create_timer(0.01, self.read_imu)

# No retry logic
def read_raw_data(self, addr):
    high = self.bus.read_byte_data(self.addr, addr)  # Fails if bus busy
    low = self.bus.read_byte_data(self.addr, addr + 1)
    # ...
```

**Problem:** One I2C error → exception → node crashes or CPU spikes trying to recover

### After (Robust Code)
```python
# Timer at 50Hz - more reasonable
self.timer = self.create_timer(0.02, self.read_imu)

# With retry logic
def read_raw_data(self, addr):
    try:
        high = self.read_byte_with_retry(addr)  # Automatic retries
        low = self.read_byte_with_retry(addr + 1)
        self.read_failure_count = 0  # Reset on success
        # ...
    except Exception as e:
        self.read_failure_count += 1
        if self.read_failure_count >= 10:
            self.init_sensor()  # Reinitialize if too many failures
```

**Benefit:** Temporary I2C glitches are handled gracefully, node stays stable

## Common Error Messages

### `OSError: [Errno 121] Remote I/O error`
**Cause:** I2C communication failed
**Fix:** Retry logic handles this automatically now

### `Failed to read MPU6050 data`
**Cause:** Transient bus error
**Fix:** Robust node retries automatically

### High CPU usage
**Cause:** Rapid retry loops without delays
**Fix:** Reduced polling rate + proper retry delays

## Testing Procedure

After applying the fix:

1. **Stop everything:**
   ```bash
   pkill -f ros2
   ```

2. **Test sensor directly:**
   ```bash
   python3 ~/frr_ws/diagnose_mpu6050.py
   ```
   Should show 10/10 successful reads

3. **Start robot system:**
   ```bash
   cd ~/frr_ws
   source install/setup.bash
   ros2 launch frr_bringup rover_bringup.launch.py
   ```

4. **Monitor in another terminal:**
   ```bash
   # Should see ~50 Hz
   ros2 topic hz /imu/mpu6050
   
   # Should see data
   ros2 topic echo /imu/mpu6050 --once
   
   # CPU should be low
   ps aux | grep mpu6050
   ```

5. **Drive the robot:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   IMU should continue working while motors are running

## Success Criteria

✅ **Fixed when you see:**
- MPU6050 node CPU usage < 10%
- Consistent 50 Hz publish rate on `/imu/mpu6050`
- IMU data continues publishing while motors are running
- No "Remote I/O error" messages in logs

## If Still Having Issues

1. **Check wiring:**
   - SDA to GPIO 2 (Pin 3)
   - SCL to GPIO 3 (Pin 5)
   - VCC to 3.3V
   - GND to GND

2. **Try lower I2C speed:**
   ```bash
   sudo nano /boot/firmware/config.txt
   # Set: dtparam=i2c_arm_baudrate=50000
   sudo reboot
   ```

3. **Check for hardware damage:**
   - Run: `sudo i2cdetect -y 1`
   - MPU6050 should appear at 0x68
   - If not detected, check wiring/power

4. **Check logs:**
   ```bash
   ros2 node info /mpu6050_node
   ros2 topic list | grep imu
   ```

## Files Modified

- **Original node (backed up):** 
  `/home/alibaba/frr_ws/src/frr_sensors/frr_sensors/mpu6050_node.py.backup`

- **New robust node:**
  `/home/alibaba/frr_ws/src/frr_sensors/frr_sensors/mpu6050_node.py`

- **Diagnostic tool:**
  `/home/alibaba/frr_ws/diagnose_mpu6050.py`

- **Fix script:**
  `/home/alibaba/frr_ws/fix_mpu6050_issue.sh`

## Restore Original (If Needed)

If you need to revert:

```bash
cd ~/frr_ws/src/frr_sensors/frr_sensors
cp mpu6050_node.py.backup mpu6050_node.py
cd ~/frr_ws
colcon build --packages-select frr_sensors
```

## Summary

The MPU6050 **hardware is working fine** (as shown by `i2cdetect` and the diagnostic script). The issue was in the **software implementation** - specifically lack of error handling when I2C communication has temporary glitches, especially when motors are running.

The robust node adds retry logic, proper delays, error recovery, and reduced polling rate to handle these real-world conditions gracefully.
