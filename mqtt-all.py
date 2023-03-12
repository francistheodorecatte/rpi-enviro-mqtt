#!/usr/bin/env python3
"""
Run mqtt broker on localhost: sudo apt-get install mosquitto mosquitto-clients

Example run: python3 mqtt-all.py --broker 192.168.1.164 --topic enviro --username xxx --password xxxx
"""

import argparse
import ST7735
import time
import ssl
import RPi.GPIO as GPIO
from bme280 import BME280
from pms5003 import PMS5003, ReadTimeoutError, SerialTimeoutError
from enviroplus import gas
from enviroplus.noise import Noise
from statistics import mean

try:
    # Transitional fix for breaking change in LTR559
    from ltr559 import LTR559

    ltr559 = LTR559()
except ImportError:
    import ltr559

from subprocess import PIPE, Popen, check_output
from PIL import Image, ImageDraw, ImageFont
from fonts.ttf import RobotoMedium as UserFont
import json

import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


DEFAULT_MQTT_BROKER_IP = "localhost"
DEFAULT_MQTT_BROKER_PORT = 1883
DEFAULT_MQTT_TOPIC = "enviroplus"
DEFAULT_READ_INTERVAL = 5
DEFAULT_TLS_MODE = False
DEFAULT_USERNAME = None
DEFAULT_PASSWORD = None
DEFAULT_OLED = True
DEFAULT_COMP_FACTOR = 2.25


# mqtt callbacks
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connected OK")
    else:
        print("Bad connection Returned code=", rc)


def on_publish(client, userdata, mid):
    print("mid: " + str(mid))


# Read values from BME280 and return as dict
def read_bme280(bme280, comp_factor):
    values = {}
    # Average temperature readings for more accuracy
    average = []
    for x in range(5):
        cpu_temp = get_cpu_temperature()
        raw_temp = bme280.get_temperature() # float
        temp_temp = raw_temp - ((cpu_temp - raw_temp) / comp_factor)
        average.append(temp_temp)
        time.sleep(0.1) # Give a little more time between readings
    # Then round to one decimal point precision
    comp_temp = round(mean(average), 1)
    values["temperature"] = comp_temp
    values["pressure"] = round(
        int(bme280.get_pressure() * 100), -1
    )  # round to nearest 10
    values["humidity"] = int(bme280.get_humidity())
    data = gas.read_all()
    values["oxidised"] = int(data.oxidising / 1000)
    values["reduced"] = int(data.reducing / 1000)
    values["nh3"] = int(data.nh3 / 1000)
    values["lux"] = int(ltr559.get_lux())
    values["uv_index"] = int(get_uv(data))
    return values


# Read values PMS5003 and return as dict
def read_pms5003(pms5003):
    values = {}
    try:
        pm_values = pms5003.read()  # int
        values["pm1"] = pm_values.pm_ug_per_m3(1)
        values["pm25"] = pm_values.pm_ug_per_m3(2.5)
        values["pm10"] = pm_values.pm_ug_per_m3(10)
    except ReadTimeoutError:
        pms5003.reset()
        pm_values = pms5003.read()
        values["pm1"] = pm_values.pm_ug_per_m3(1)
        values["pm25"] = pm_values.pm_ug_per_m3(2.5)
        values["pm10"] = pm_values.pm_ug_per_m3(10)
    return values


# Get CPU temperature to use for compensation
def get_cpu_temperature():
    process = Popen(
        ["vcgencmd", "measure_temp"], stdout=PIPE, universal_newlines=True
    )
    output, _error = process.communicate()
    return float(output[output.index("=") + 1:output.rindex("'")])


# Get UV Index
def get_uv(data):
    """
    Read ADC #3 input, then divide by default Enviro+ gain (6.148), and multiply by 10.
    This will give a rough UV Index value based on the mV output of the GUVA-S12SD.
    You may need to change the UV Index calculation for your analog UV sensor.
    """
    uv_index = round((int(data.adc) / 6.148) * 10)
    if uv_index > 11:
        # Ignore any index value over 11
        return '11'
    else:
        return uv_index


# Get Noise Profile and return as dict
def get_noise_profile(noise):
    values = {}
    low, mid, high, amp = noise.get_noise_profile()
    values["noise_low"] = round(low * 128, 1)
    values["noise_mid"] = round(mid * 128, 1)
    values["noise_high"] = round(high * 128, 1)
    values["noise_total"] = round(amp * 64, 1)
    return values


# Get Raspberry Pi serial number to use as ID
def get_serial_number():
    with open("/proc/cpuinfo", "r") as f:
        for line in f:
            if line[0:6] == "Serial":
                return line.split(":")[1].strip()


# Check for Wi-Fi connection
def check_wifi():
    if check_output(["hostname", "-I"]):
        return True
    else:
        return False


# Display Raspberry Pi serial and Wi-Fi status on LCD
def display_status(disp, mqtt_broker):
    # Width and height to calculate text position
    WIDTH = disp.width
    HEIGHT = disp.height
    # Text settings
    font_size = 12
    font = ImageFont.truetype(UserFont, font_size)

    wifi_status = "connected" if check_wifi() else "disconnected"
    text_colour = (255, 255, 255)
    back_colour = (0, 170, 170) if check_wifi() else (85, 15, 15)
    device_serial_number = get_serial_number()
    message = "{}\nWi-Fi: {}\nmqtt-broker: {}".format(
        device_serial_number, wifi_status, mqtt_broker
    )
    img = Image.new("RGB", (WIDTH, HEIGHT), color=(0, 0, 0))
    draw = ImageDraw.Draw(img)
    size_x, size_y = draw.textsize(message, font)
    x = (WIDTH - size_x) / 2
    y = (HEIGHT / 2) - (size_y / 2)
    draw.rectangle((0, 0, 160, 80), back_colour)
    draw.text((x, y), message, font=font, fill=text_colour)
    disp.display(img)


def main():
    parser = argparse.ArgumentParser(
        description="Publish enviroplus values over mqtt"
    )
    parser.add_argument(
        "--broker",
        default=DEFAULT_MQTT_BROKER_IP,
        type=str,
        help="mqtt broker IP",
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_MQTT_BROKER_PORT,
        type=int,
        help="mqtt broker port",
    )
    parser.add_argument(
        "--topic", default=DEFAULT_MQTT_TOPIC, type=str, help="mqtt topic"
    )
    parser.add_argument(
        "--interval",
        default=DEFAULT_READ_INTERVAL,
        type=int,
        help="the read interval in seconds",
    )
    parser.add_argument(
        "--tls",
        default=DEFAULT_TLS_MODE,
        action='store_true',
        help="enable TLS"
    )
    parser.add_argument(
        "--username",
        default=DEFAULT_USERNAME,
        type=str,
        help="mqtt username"
    )
    parser.add_argument(
        "--password",
        default=DEFAULT_PASSWORD,
        type=str,
        help="mqtt password"
    )
    parser.add_argument(
        "--oled",
        default=DEFAULT_OLED,
        action=argparse.BooleanOptionalAction,
        help="enable or disable display"
    )
    parser.add_argument(
        "--comp_factor",
        default=DEFAULT_COMP_FACTOR,
        type=float,
        help="temperature compensation factor"
    )
    args = parser.parse_args()

    # Raspberry Pi ID
    device_serial_number = get_serial_number()
    device_id = "raspi-" + device_serial_number

    print(
        f"""mqtt-all.py - Reads Enviro plus data and sends over mqtt.

    broker: {args.broker}
    client_id: {device_id}
    port: {args.port}
    topic: {args.topic}
    interval: {args.interval}
    tls: {args.tls}
    username: {args.username}
    password: {args.password}
    oled: {args.oled}
    comp_factor: {args.comp_factor}

    Press Ctrl+C to exit!

    """
    )

    mqtt_client = mqtt.Client(client_id=device_id)
    if args.username and args.password:
        mqtt_client.username_pw_set(args.username, args.password)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_publish = on_publish

    if args.tls is True:
        mqtt_client.tls_set(tls_version=ssl.PROTOCOL_TLSv1_2)

    if args.username is not None:
        mqtt_client.username_pw_set(args.username, password=args.password)

    mqtt_client.connect(args.broker, port=args.port)

    bus = SMBus(1)

    # Create BME280 instance
    bme280 = BME280(i2c_dev=bus)

    # Setup ADC#3
    gas.setup()
    gas.enable_adc(True)

    # Create Noise instance
    # The measurement duration also (indirectly) sets our publish interval
    noise = Noise(duration=args.interval)

    # Create LCD instance
    disp = ST7735.ST7735(
        port=0, cs=1, dc=9, backlight=12, rotation=270, spi_speed_hz=10000000
    )

    if args.oled is True:
        # Initialize display
        disp.begin()
    elif args.oled is False:
        # Initialize display (even though it's being disabled below)
        disp.begin()
        # Blank display
        disp.command(0x28)
        # Turn off the display backlight (this function is broken in the ST7735 library)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(12, GPIO.OUT)
        GPIO.output(12, GPIO.LOW)

    # Try to create PMS5003 instance
    HAS_PMS = False
    try:
        pms5003 = PMS5003()
        _ = pms5003.read()
        HAS_PMS = True
        print("PMS5003 sensor is connected")
    except SerialTimeoutError:
        print("No PMS5003 sensor connected")

    # Display Raspberry Pi serial and Wi-Fi status
    print("RPi serial: {}".format(device_serial_number))
    print("Wi-Fi: {}\n".format("connected" if check_wifi() else "disconnected"))
    print("MQTT broker IP: {}".format(args.broker))

    # Main loop to read data, display, and send over mqtt
    mqtt_client.loop_start()
    while True:
        try:
            values = read_bme280(bme280, args.comp_factor)
            noise_values = get_noise_profile(noise)
            values.update(noise_values)
            if HAS_PMS:
                pms_values = read_pms5003(pms5003)
                values.update(pms_values)
            values["serial"] = device_serial_number
            print(values)
            mqtt_client.publish(args.topic, json.dumps(values), retain=True)
            if args.oled is True:
                display_status(disp, args.broker)
            #time.sleep(args.interval)
        except Exception as e:
            print(e)


if __name__ == "__main__":
    main()
