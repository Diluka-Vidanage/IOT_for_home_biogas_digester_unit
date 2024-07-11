#Created by Diluka Vidanage

import os
import network
import time
import ufirebase as firebase
import machine
import onewire
import ds18x20
from rotary_irq_esp import RotaryIRQ
from machine import Pin, SoftI2C, PWM
import ssd1306
from bmp180 import BMP180

# Function to connect to Wi-Fi
def connect_wifi(ID,Password):
    if oled:
        oled.fill(0)
        oled.text('Connecting to WiFi', 0, 0)
        oled.show()

    GLOB_WLAN = network.WLAN(network.STA_IF)
    GLOB_WLAN.active(True)
    GLOB_WLAN.connect(ID, Password)

    while not GLOB_WLAN.isconnected():
        pass

    if oled:
        oled.fill(0)
        oled.text('WiFi Connected', 0, 0)
        oled.show()
        time.sleep(1)

# Initialize rotary encoder
def init_rotary_encoder(clk_pin, dt_pin):
    if oled:
        oled.fill(0)
        oled.text('Initializing Rotary', 0, 0)
        oled.show()

    return RotaryIRQ(
        pin_num_clk=clk_pin,
        pin_num_dt=dt_pin,
        reverse=False,
        incr=1,
        range_mode=RotaryIRQ.RANGE_UNBOUNDED,
        pull_up=True,
        half_step=False,
    )

# Initialize DS18B20 sensor
def init_ds18b20(pin_num):
    if oled:
        oled.fill(0)
        oled.text('Initializing DS18B20', 0, 0)
        oled.show()

    try:
        ds_pin = machine.Pin(pin_num)
        ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
        roms = ds_sensor.scan()
        if len(roms) == 0:
            raise Exception("No DS18B20 devices found")
        return ds_sensor, roms
    except Exception as e:
        print(f"Error initializing DS18B20: {e}")
        return None, None

# Initialize I2C and OLED display
def init_oled(i2c_scl, i2c_sda, oled_address, oled_width, oled_height, retries=5):
    global oled, i2c  # Declare oled and i2c as global variables

    for _ in range(retries):
        try:
            i2c = SoftI2C(scl=Pin(i2c_scl), sda=Pin(i2c_sda), freq=100000)
            oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c, addr=oled_address)
            oled.fill(0)
            oled.text('OLED Initialized', 0, 0)
            oled.show()
            return oled, i2c
        except Exception as e:
            print(f"Error initializing OLED: {e}")
            time.sleep(1)  # Retry after a short delay

    return None, None

# Initialize BMP180 sensor
def init_bmp180(i2c):
    if oled:
        oled.fill(0)
        oled.text('Initializing BMP180', 0, 0)
        oled.show()

    while True:
        try:
            bmp180 = BMP180(i2c)
            bmp180.oversample_sett = 2
            bmp180.baseline = 101325
            return bmp180
        except OSError as e:
            print(f"OSError: {e}")
            print("Retrying in 1 second...")
            time.sleep(1)

# Read temperature from DS18B20
def read_ds18b20(ds_sensor, roms):
    try:
        ds_sensor.convert_temp()
        time.sleep(1)
        return ds_sensor.read_temp(roms[0])
    except IndexError as e:
        print(f"Error reading DS18B20: {e}")
        return "Error"
    except Exception as e:
        print(f"Unexpected error reading DS18B20: {e}")
        return "Error"

# Read data from BMP180
def read_bmp180(bmp180):
    try:
        temp = bmp180.temperature
        pressure = bmp180.pressure - 101325
        altitude = bmp180.altitude
        return temp, pressure, altitude
    except OSError as e:
        print(f"OSError reading BMP180: {e}")
        return "Error", "Error", "Error"
    except Exception as e:
        print(f"Unexpected error reading BMP180: {e}")
        return "Error", "Error", "Error"

# Calculate RPM from encoder steps
def calculate_rpm(steps, interval_seconds):
    rotations = steps / 400  # 400 steps per rotation
    rpm = (rotations / interval_seconds) * 60  # Convert to RPM
    return rpm

# Update OLED display
def update_oled(oled, ds_temp, bmp_temp, pressure, altitude, rpm):
    if oled:
        oled.fill(0)  # Clear the screen
        try:
            oled.text('BIO GAS DIGESTER', 0, 0)
            oled.text('G21', 0, 10)
            if ds_temp != "Error":
                oled.text(f'Slurry Temp: {ds_temp:.2f} C', 0, 20)
            else:
                oled.text('DS18B20 Temp: Error', 0, 20)

            if bmp_temp != "Error":
                oled.text(f'Gas Temp: {bmp_temp:.2f} C', 0, 30)
            else:
                oled.text('BMP180 Temp: Error', 0, 30)

            if pressure != "Error":
                oled.text(f'Pressure: {pressure} Pa', 0, 40)
            else:
                oled.text('Pressure: Error', 0, 40)
                
            if rpm != "Error":
                oled.text(f'RPM: {rpm:.2f}', 0, 50)
            else:
                oled.text('RPM: Error', 0, 50)
        except Exception as e:
            print(f"Error updating OLED: {e}")
        oled.show()

# Update parameters in the database
def update_parameters(ds_temp, bmp_temp, pressure, rpm, mode):
    data = {
        "Temperature (DS18B20) (C)": ds_temp,
        "Temperature (BMP180) (C)": bmp_temp,
        "Pressure (Pa)": pressure,
        "Motor speed (rpm)": rpm,
        "Mode": mode,
        "Setpoint": setpoint
    }
    try:
        firebase.put("Parameters", data)  # Removed bg=0
    except TypeError as e:
        print(f"TypeError: {e}")
    except Exception as e:
        print(f"Unexpected error updating Firebase: {e}")

# PID control function
def pid_control(current_rpm):
    global previous_error, integral, servo_angle

    error = setpoint - current_rpm
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error

    # Adjust servo angle based on PID output
    servo_angle += output

    # Limit servo angle to between 0 and 180 degrees
    if servo_angle > 180:
        servo_angle = 180
    elif servo_angle < 0:
        servo_angle = 0

    # Convert angle to PWM duty
    duty = int((servo_angle / 180) * 1023)
    servo_pwm.duty(duty)


def main():
    # Firebase setup
    firebase.setURL("") #Put database URL here

    # Pin definitions
    PIN_DS18B20 = 4
    PIN_ROTARY_CLK = 13
    PIN_ROTARY_DT = 12
    PIN_I2C_SCL = 22
    PIN_I2C_SDA = 21

    # Initialize PID control variables
    Kp = 0.1
    Ki = 0
    Kd = 0

    setpoint = 68  # Default setpoint value (50-100)

    servo_angle = 0
    servo_pin = 15  
    servo_pwm = PWM(Pin(servo_pin), freq=50)

    previous_error = 0
    integral = 0
    
    while True:
        try:
            # Connect to WiFi and initialize components
            connect_wifi('','') #Wifi ID & Password here

            oled, i2c = init_oled(PIN_I2C_SCL, PIN_I2C_SDA, oled_address=0x3C, oled_width=128, oled_height=64)
            rotary_encoder = init_rotary_encoder(PIN_ROTARY_CLK, PIN_ROTARY_DT)
            ds_sensor, roms = init_ds18b20(PIN_DS18B20)
            bmp180 = init_bmp180(i2c)

            val_old = rotary_encoder.value()
            steps = 0
            interval_seconds = 2
            last_time = time.time()

            while True:
                try:
                    # Read rotary encoder value
                    val_new = rotary_encoder.value()
                    steps += abs(val_new - val_old)
                    val_old = val_new

                    # Check if it's time to update the display
                    current_time = time.time()
                    if current_time - last_time >= interval_seconds:
                        # Calculate RPM
                        rpm = calculate_rpm(steps, interval_seconds)
                        steps = 0  # Reset step counter

                        # Read sensor data
                        ds_temp = read_ds18b20(ds_sensor, roms) if ds_sensor and roms else "Error"
                        bmp_temp, pressure, altitude = read_bmp180(bmp180) if bmp180 else ("Error", "Error", "Error")

                        # Update display
                        update_oled(oled, ds_temp, bmp_temp, pressure, altitude, rpm)

                        # Update parameters on Firebase
                        update_parameters(ds_temp, bmp_temp, pressure, rpm, mode)

                        # Retrieve setpoint and mode from Firebase
                        try:
                            setpoint_response = firebase.get("Parameters/Setpoint")
                            if setpoint_response is not None:
                                setpoint = setpoint_response

                            mode_response = firebase.get("Parameters/Mode")
                            if mode_response is not None:
                                mode = mode_response
                        except Exception as e:
                            print(f"Error reading from Firebase: {e}")

                        # Perform PID control and motor adjustment
                        current_rpm = rpm  # Use calculated RPM for control
                        pid_control(current_rpm)

                        last_time = current_time

                    # Sleep for a short duration to reduce CPU usage
                    time.sleep(0.1)

                except Exception as e:
                    print(f"Unexpected error in main loop: {e}")
                    if oled:
                        oled.fill(0)
                        oled.text('Error in main loop', 0, 0)
                        oled.text(str(e), 0, 10)
                        oled.show()
                    time.sleep(1)  # Pause before retrying

        except Exception as e:
            print(f"Critical error: {e}")
            if oled:
                oled.fill(0)
                oled.text('Restarting...', 0, 0)
                oled.show()
            time.sleep(1)  # Pause before restarting
            machine.reset()  # Restart the pins
            main() #Restart the main program.



if __name__ == "__main__":
    main()
