from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
from tkinter import *
import time
import paho.mqtt.client as mqtt
import queue as q
import sys
from threading import Thread

#Variables
ratio_mm = 49.7030
ratio_inch = 25.4*ratio_mm
ratio = ratio_inch
min_recorded = 0.001
unit = " in"
formatting = ".3f"

connection_messages = q.Queue()
Disconnected = True
count = 0
checkcycles = 0
Diff_Value = -1

def on_disconnect(self,userdata,rc):
    global Disconnected
    mqttClient.loop_stop()
    print("Client Disconnected")
    Disconnected = True
    status["text"]="Disconnected"
    status["fg"]="red"

def on_connect(self,userdata,flags,result):
    global Disconnected
    print("Connected to Mqtt broker")
    Disconnected = False
    ### update the label to say connected
    status["text"] = "Connected"
    status["fg"]="green"

def terminate():
    """ Quits the program """
    sys.exit("Program Exited")

def connect():
    """Attempts to the MQTT broker""" 
    try:
        mqttClient.connect("192.168.7.1")
        mqttClient.subscribe("stringGauge/inputs")
        mqttClient.subscribe("stringGauge/connection")
        mqttClient.loop_start()
    except:
        print("Couldnt Connect")
        return

def on_message(client,userdata,message):
    mesValue = message.payload.decode("utf-8")
    if mesValue.lower() == "zero":
        zero()
    
    elif mesValue.lower() == "units":
        unitSwitch()
    
    elif mesValue.lower() == "connected":
        connection_messages.put(mesValue)
        print("connection message recieved")
def marginally_different(val):
    """Checks if value is different from global variable: Diffval
        Input: a float
        Return: True if value is marginally different"""
    global Diff_Value

    if abs(val-Diff_Value) > min_recorded:
        Diff_Value = val
        return True
    else: 
        return False

def update():
    """
    Main loop for getting and updating value from the string gauge
    """
    global count, Disconnected, checkcycles
    #Gets Value and Updates main Tkinter label
    variable = (encoder.getPosition()/ratio)
    formatted = format(variable,formatting)
    value["text"] = formatted +unit

    post = marginally_different(variable)
    
    if (not Disconnected) and recording.get() and post:
        mqttClient.publish("stringGauge/recording",formatted)


    if not Disconnected:
        checkcycles += 1
        if checkcycles == 100:
            mqttClient.publish("stringGauge/connection","connected")
            print("Attempting to send da message")
        elif checkcycles == 200:
            checkcycles = 0
            print(connection_messages.empty())

            if connection_messages.empty():
               on_disconnect(True,True,True)
            else:
                connection_messages.get()

    elif Disconnected and count == 600:
        connection_thread = Thread(target=connect)
        connection_thread.start()
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
    global min_recorded
    
    if ratio == ratio_mm:
        ratio = ratio_inch
        unit = " in"
        formatting = ".3f"
        value.config(font=("Helvetica",95))
        min_recorded = 0.00001
    else:
        ratio = ratio_mm
        unit = " mm"
        formatting = ".2f"
        value.config(font=("Helvetica",80))
        min_recorded(0.0001)

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
value.place(relx=0.5,rely=0.2, anchor="n")

#Connection Status
status = Label(main, text="Disconnected")
status.config(font=("Helvetica",20),fg="red")
status.place(relx=0.75,rely=0.83)

#Zero button
zeroButton = Button(main, padx = 10, pady = 10, command = zero, text = "Zero")
zeroButton.config(font=("Helvetica",50))
zeroButton.place(relx = 0.15, rely= 0.6, anchor = "nw")

#Switchunits Button
unitsButton = Button(main, padx = 10, pady = 10, command = unitSwitch, text = "mm/inch")
unitsButton.config(font=("Helvetica",50))
unitsButton.place(relx = 0.85, rely= 0.6, anchor = "ne")

#Terminate Button
terminateButton = Button(main, command = terminate, text = "X")
terminateButton.config(font=("Helvetica",20))
terminateButton.place(x=0,y=0)

#Record Data Swtich
recording = BooleanVar(main)
recording.set(True)
recordValues = Checkbutton(main, text="Record Data", variable=recording)
recordValues.place(relx=0.70,rely=0.9)
recordValues.config(font=("Helvetica",20))

#mqtt things
mqttClient = mqtt.Client("stringGauge")
mqttClient.on_message = on_message
mqttClient.on_disconnect = on_disconnect
mqttClient.on_connect = on_connect
t = Thread(target = connect)
t.start()

#Updates live readingss
update()

#starts program
mainloop()