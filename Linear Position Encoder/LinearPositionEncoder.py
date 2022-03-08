from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
from tkinter import *
import time as t
import paho.mqtt.client as mqtt

#Variables
ratio_mm = 49.7030
ratio_inch = 25.4*ratio_mm
ratio = ratio_inch
unit = " in"
formatting = ".3f"

#For connecting to mqtt
Disconnected = True
update_time = 0

#update encoder flags
encoder0_connected = False
encoder1_connected = False
encoder2_connected = False
encoder3_connected = False


def on_disconnect():
    """ 
    Sets disconnect flag for mqtt broker when disconnected
    """
    global Disconnected
    mqttClient.loop_stop()
    print("Client Disconnected")
    Disconnected = True

def on_connect():
    """
    Sets disconnect flag for mqtt broker when connected 
    """

    global Disconnected
    Disconnected = False

def on_message(client,userdata,message):
    """
    calls either zero or units function when messages are recieved
    """
    mesValue = message.payload.decode("utf-8")

    if mesValue.lower() == "zero":
        zero()
    
    elif mesValue.lower() == "units":
        unitSwitch()

def connect():
    """
    Attempts to connect to mqtt broker
    """
    try:
        mqttClient.connect("192.168.7.1")
        mqttClient.subscribe("encoder0/inputs")
        mqttClient.loop_start()
    except:
        print("Could not connect to 92.168.7.1")
        return

def new_encoder(position):
    """
    Creates a single encoder and sets the channel
    """

    temp_encoder = encoder()
    temp_encoder.setChannel(position)
    temp_encoder.setPosition(0)
    temp_encoder.openWaitForAttachment(1000)
    return temp_encoder

def check_encoders():
    """
    Attempts to attach an encoder on all 3 channels
    Will assign objects to encoder0, encoder1, encoder2, or encoder3 based on channel
    """

    global encoder0, encoder1, encoder2, encoder3
    global encoder0_connected, encoder1_connected, encoder2_connected, encoder3_connected
    
    if encoder0_connected == False:
        try:
            encoder0 = new_encoder(0)
            encoder0_connected = True
        except:
            print("No encoder found on channel 0")

    if encoder1_connected == False:
        try:
            encoder1 = new_encoder(1)
            encoder1_connected = True
        except:
            print("No encoder found on channel 1")

    if encoder2_connected == False:
        try:
            encoder2 = new_encoder(2)
            encoder2_connected = True
        except:
            print("No encoder found on channel 2")

    if encoder3_connected == False:
        try:
            encoder3 = new_encoder(3)
            encoder3_connected = True
        except:
            print("No encoder found on channel 3")


def update():
    """
    Main loop for getting and updating value from the string gauge
    """
    #if encoders are not connected wait for an encoder to connect
    if encoder0_connected == False and encoder1_connected == False and encoder2_connected == False and encoder3_connected == False:
        print("waiting for encoder")
        check_encoders()
        t.sleep(4)
        main.after(50, update)
    
    #main function loop
    else:
        global count
        #Gets Value and Updates main Tkinter label
        variable = (encoder0.getPosition()/ratio)
        formatted = format(variable,formatting)
        value["text"] = formatted +unit

        if not Disconnected:
            mqttClient.publish("encoder0",variable)
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
    encoder0.setPosition(0)


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
#creates the mqtt client object
mqttClient = mqtt.Client("encoder0")
mqttClient.on_message = on_message
mqttClient.on_disconnect = on_disconnect
mqttClient.on_connect = on_connect
connect()

#Encoder set-up
#
check_encoders()

##########################
# Tkinter options
#
#
#
#
##########################


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




