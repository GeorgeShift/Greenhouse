import machine
from machine import UART
from umqtt.simple import MQTTClient
import network

# nastavení wifi
SSID = "Xperiator"
wifi_password = "fullstall"

# nastavení thingspeaku
SERVER = "mqtt.thingspeak.com"
CHANNEL_ID = "466923"
WRITE_API_KEY = "WCUULPOY6B03QP90"
topic = "channels/" + CHANNEL_ID + "/publish/" + WRITE_API_KEY

# proměnná pro uložení dat z uartu
data = b""
# hodnota odporu na děliči u termistoru
rezistor = 1000
# inicializace proměnných
cerpadlo = 0
zmena = 0
teplota = 25


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


client = MQTTClient("umqtt_client", SERVER)
wifi1 = Wifi(SSID, wifi_password)
uart = UART(0, 9600)
uart.init(9600, bits=8, parity=None, stop=1)

while(True):
    # Připojení k WiFi pokud je odpojeno
    if not wifi1.connected():
        print("Připojování k WiFi.")
        wifi1.connect()
        print("Připojeno k WiFi:", wifi1.ssid)

    # kontrola přitomnosti dat na uartu
    if uart.any():
        data = uart.readline()
        data = data.decode('UTF-8')
        data = data.strip()

        if "STOP" in data:
            print("Nádrž plná - vypnutí čerpadla")
            cerpadlo = 0
            zmena = 1

        elif "START" in data:
            print("Nádrž prázdná - zapnutí čerpadla")
            cerpadlo = 1
            zmena = 1

        elif data[0] == "T":
            data = data[1: len(data)]

            try:
                ad = int(data)
            except Exception as e:
                print("Chyba:", type(e).__name__)
                pass
            else:
                # výpočet napětí na AD předovníku
                napeti = (ad * 5.04) / 1024
                # výpočet odporu termistoru
                odpor = (rezistor * napeti) / (5.04 - napeti)
                # výpočet a zaokrohlení přibližné teploty
                teplota = 0.1367 * odpor - 112.69
                teplota = round(teplota, 1)

                # odeslání dat na thingspeak
                payload = "field1=" + str(teplota)
                payload += "&field2=" + str(cerpadlo)
                client.connect()
                client.publish(topic, payload)
                client.disconnect()

                # výpis naměřených hodnot do replu
                print("AD:", ad)
                print("Napeti:", napeti)
                print("Odpor:", odpor)
                print("Teplota:", teplota)
                print()

        if zmena:
            zmena = 0
            # odeslání dat na thingspeak
            payload = "field1=" + str(teplota)
            payload += "&field2=" + str(cerpadlo)
            client.connect()
            client.publish(topic, payload)
            client.disconnect()

# reset v případě že program dojde až sem
print("Hard reset")
machine.reset()
