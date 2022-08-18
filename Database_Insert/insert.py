#Creates a csv file from mqtt data and adds it to the database

#Get test name (Ideally date + name from command line)
#get test type from command line
#connect to sql
#create table
#connect to mqtt
#How do we know when test is over (mqtt command maybe?)
#close db and end program

import sys
from datetime import datetime as dt
import paho.mqtt.client as mqtt
import sqlite3 as sql

#collect command line arguments
starting_datetime = dt.now().strftime("%d/%m/%Y-%H:%M:%S")
test_name = f"{sys.argv[0]}_{starting_datetime}"
test_type = sys.argv[1]

def generate_sql(test, function, data=[]):
    """Generates strings of SQL to be executed by the cursor object
       test: the type of test the script is recording as a string
       function: either the string "create" or "insert" depending on what sql needs to be generated
       return: A string of sql to be executed
    """
    if test == "big_bertha":
        if function == "create":
            create_statement = "CREATE TABLE " + table_name + " (ID INTEGER PRIMARY KEY AUTOINCREMENT, " \
                                "time DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M%f', 'NOW', 'localtime')), " \
                                "jog_setpoint REAL, position REAL, force_setpoint INTEGER, force INTEGER, " \
                                "pressure_t INTEGER, pressure_c INTEGER)"
            return create_statement
        else:
            insert_statement = f"INSERT INTO {table_name} (jog_setpoint,position,force_setpoint,force,pressure_t,pressure_c) VALUES({data[0]},{data[1]},{data[2]},{data[3]},{data[4]},{data[5]})"
            return insert_statement

    elif test == "string_gauge":
        if function == "create":
            create_statement = "CREATE TABLE " + table_name + " (ID INTEGER PRIMARY KEY AUTOINCRIMENT, " \
                                    "time DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M:%f', 'NOW', 'localtime')), " \
                                    "displacement REAL)" 
            return create_statement
        else:
            insert_statement = f"INSERT INTO {table_name} (displacement) VALUES({data[0]})"
            return insert_statement

    elif test == "phenix_rts":
        if function == "create":
            create_statement = "CREATE TABLE " + table_name + " (ID INTEGER PRIMARY KEY AUTOINCREMENT, " \
                                    "time DATETIME DEFAULT(STRFTIME('%Y-%m-%d %H:%M:%f', 'NOW', 'localtime')) " \
                                    "voltage REAL, current REAL)" 
            return create_statement
        else:
            insert_statement = "INSERT INTO {table_name} (voltage, current) VALUES({data[0]},{data[1]})"
            return insert_statement

#Add future test types here as elif satements

#MQTT callbacks
def on_message(client, userdata, msg):
    print(msg.topic+str(msg.payload))
    if msg.topic == "end":
        con.close()
        end_msg = "Test Finished at: " + str(dt.now().strftime("%d/%m/%Y-%H:%M:%S"))
        client.publish("Important","Test Finished at: " + end_msg)
        sys.exit()
    else:
        payload_data = msg.payload.split(",")
        con.execute(generate_sql(test_type, "insert"))
        
def on_connect(client,userdata,flags,rc):
    client.subscribe("end")
    client.subscribe(test_type)
    client.publish("Important", "Connected and Recording")

#create sql connection and cursor object
con = sql.connect("testlab.db")
cur = con.cursor()

#create table for this test
cur.execute(generate_sql(test_type, "create"))

#connect to mqtt 
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

#run program
client.connect("192.168.50.10",1883)
client.loop_forever()