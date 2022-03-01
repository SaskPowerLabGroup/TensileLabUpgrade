import paho.mqtt.client as mqtt #import the client1
import time as t
############
def on_disconnect(client, userdata, rc=0):
    mqttClient.loop_stop()
    print("Client Disconnected")

def on_connect():
    print("Connected OK")

def on_message(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)
########################################
broker_address="192.168.7.1"
#broker_address="iot.eclipse.org"
print("creating new instance")
client = mqtt.Client("P1") #create new instance
client.on_message=on_message #attach function to callback
client.on_disconnect = on_disconnect #attach function to callback
client.on_connect = on_connect()
print("connecting to broker")
client.connect(broker_address) #connect to broke

client.subscribe("stringGauge/inputs")
client.loop_forever() #start the loop


