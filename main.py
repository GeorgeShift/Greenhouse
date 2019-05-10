import machine
from machine import UART
from umqtt.simple import MQTTClient
import network


class Wifi:
    """ Třída pro práci s wifi připojením """
    def __init__(self, ssid, password):
        self.ssid = ssid
        self.password = password

    def connect(self):
        sta_if = network.WLAN(network.STA_IF)

        if not sta_if.isconnected():
            sta_if.active(True)
            sta_if.connect(self.ssid, self.password)
            while not sta_if.isconnected():
                pass

    def connected(self):
        sta_if = network.WLAN(network.STA_IF)
        if not sta_if.isconnected():
            return False
        else:
            return True


data = b""
wifi1 = Wifi("Xperiator", "fullstall")
uart = UART(0, 9600)
uart.init(9600, bits=8, parity=None, stop=1)

SERVER = "mqtt.thingspeak.com"
client = MQTTClient("umqtt_client", SERVER)
CHANNEL_ID = "466923"
WRITE_API_KEY = "WCUULPOY6B03QP90"
topic = "channels/" + CHANNEL_ID + "/publish/" + WRITE_API_KEY

while(True):
    if not wifi1.connected():
        print("Connecting to wifi.")
        wifi1.connect()
        print("Connected to WiFi:", wifi1.ssid)

    if uart.any():
        data = uart.readline()
        data = data.decode('UTF-8')
        # data = data.strip()

        try:
            ad = int(data)
        except ValueError as e:
            print("Chyba:", type(e).__name__)
            pass
        else:
            ad = ad - 35
            teplota = 0.1963*ad - 135.76
            teplota = round(teplota, 1)
            payload = "field1="+str(teplota)
            client.connect()
            client.publish(topic, payload)
            client.disconnect()

            print("AD:", ad)
            print("Teplota:", teplota)

print("Hard reset")
machine.reset()
