from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
from tkinter import *
import time
import paho.mqtt.client as mqtt

#Variables
ratio_mm = 49.7030
ratio_inch = 25.4*ratio_mm
ratio = ratio_inch
unit = " in"
formatting = ".3f"
Disconnected = True
update_time = 0

def on_disconnect():
    global Disconnected
    mqttClient.loop_stop()
    print("Client Disconnected")
    Disconnected = True

def on_connect():
    global Disconnected
    Disconnected = False

def connect():
    try:
        mqttClient.connect("192.168.7.1")
        mqttClient.subscribe("stringGauge/inputs")
        mqttClient.loop_start()
    except:
        print("Could not connect")
        return

def on_message(client,userdata,message):
    mesValue = message.payload.decode("utf-8")

    if mesValue.lower() == "zero":
        zero()
    
    elif mesValue.lower() == "units":
        unitSwitch()

def update():
    """
    Main loop for getting and updating value from the string gauge
    """
    global count
    #Gets Value and Updates main Tkinter label
    variable = (encoder.getPosition()/ratio)
    formatted = format(variable,formatting)
    value["text"] = formatted +unit

    if not Disconnected:
        mqttClient.publish("stringGauge",variable)
    elif Disconnected and count == 1200:
        connect()
        count = 0
    else:
        count += 1

    #runs function again after 50 ms
    main.after(50, update)

def zero():
    """
    Sets encoder position to 0.00
    """
    encoder.setPosition(0)


def unitSwitch():
    """
    Switch units from inches to mm.
    Built to be called by either a message fuction or tkinter button
    """
    global ratio
    global unit
    global formatting
    
    if ratio == ratio_mm:
        ratio = ratio_inch
        unit = " in"
        formatting = ".3f"
        value.config(font=("Helvetica",95))
        
    else:
        ratio = ratio_mm
        unit = " mm"
        formatting = ".2f"
        value.config(font=("Helvetica",80))

#mqtt things
mqttClient = mqtt.Client("stringGauge")
mqttClient.on_message = on_message
mqttClient.on_disconnect = on_disconnect
mqttClient.on_connect = on_connect
connect()

#Encoder set-up
encoder = Encoder()
encoder.setChannel(1)
encoder.openWaitForAttachment(5000)
encoder.setPosition(0)

#Tkinter options
main = Tk()
main.attributes("-fullscreen", True)

#Main Value
value = Label(main, text="0.0",padx = 100)
value.config(font=("Helvetica",95))
value.grid(row = 1, column = 0, columnspan = 3, pady = 75,padx = 10)

#Zero button
zeroButton = Button(main, padx = 30, pady = 30, command = zero, text = "Zero")
zeroButton.config(font=("Helvetica",50))
zeroButton.grid(row = 2, column = 0, padx = 70)

#Switchunits Button
unitsButton = Button(main, padx = 30, pady = 30, command = unitSwitch, text = "mm/inch")
unitsButton.config(font=("Helvetica",50))
unitsButton.grid(row = 2, column = 2, padx = 50)

#Updates live readings
update()

mainloop()




