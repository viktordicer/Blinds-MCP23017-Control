import blind as bld
import smbus
import sys
import getopt
import time 
import paho.mqtt.client as mqtt
import threading
from array import *
from datetime import datetime
import os


"""
All blinds connect to the expand modules MCP23017

    first device
    1 - Living
    2 - Kitchen
    3 - Terrace
    4 - Bed
    5 - Office

    Second device
    6 - Child1 
    7 - Child2
    8 - Child3
"""


i2c_addr = [0x20, 0x21] #I2C address for devices
i2c_register_out = [0x00, 0x01] #Set A and B to output
i2c_register = [0x12, 0x13] # 0x012 is A register and 0x13 is B register
motor_delay = 0.4 # delay between stop and command for motors

Broker = "192.168.0.121"
sub_topic = ["sensor/blind/all"] 


bus = smbus.SMBus(1)

blinds = [] # list of objects all blinds
#Create new objects of blinds
#dev1
living = bld.Blind(42.7,42.3,1.2,0,1)
kitchen = bld.Blind(42.7,42.3,1.2,0,2)
terrace = bld.Blind(42.7,42.3,1.2,0,4)
bed     = bld.Blind(42.7,42.3,1.2,0,8)
office = bld.Blind(42.7,42.3,1.2,0,16)
#dev2
child1 = bld.Blind(42.7,42.3,1.2,1,1)
child2 = bld.Blind(42.7,42.3,1.2,1,2)
child3 = bld.Blind(42.7,42.3,1.2,1,4)

# add all blinds to the list
#dev1
blinds.append(living)
blinds.append(kitchen)
blinds.append(terrace)
blinds.append(bed)
blinds.append(office)
#dev2
blinds.append(child1)
blinds.append(child2)
blinds.append(child3)


##	SET REGISTER AS OUTPUTS AND CLEAR ALL REGISTERS
for dev in i2c_addr:
    for reg_out in i2c_register_out:
        bus.write_byte_data(dev,reg_out,0x00) # Set all of bank A and B to outputs 
    for reg in i2c_register:
        bus.write_byte_data(dev,reg,0) # Clear register A and B


def blind_ctr():
    while 1:
        for i in blinds:
            time_now = time.time()
            if i.movement != 'stop':
                if time_now - i.last_run > i.duration:
                    reg_A = read_data(i2c_addr[i.device], i2c_register[0])
                    reg_B = read_data(i2c_addr[i.device], i2c_register[1])
                    write_data(i2c_addr[i.device],i2c_register[0],bld.clear_bit(reg_A,int(i.bit))) #clear particular bit
                    write_data(i2c_addr[i.device],i2c_register[1],bld.clear_bit(reg_B,int(i.bit)))
                    i.stop()
                    print(i.position)

#read data from register
def read_data(device,register):
    return bus.read_byte_data(device, register)

#write data to the particular bit
def write_data(device,register,bit):
    bus.write_byte_data(device,register,bit)



def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    for i in sub_topic:
        client.subscribe(i)


def on_message(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    send_command(message)



def send_command(message):
    comm, level, blind1, blind2 = bld.decode_command(message)
    list_of_blinds = bld.find_index(blind1+blind2)
    reg_A=[]
    reg_B=[]
    for addr in i2c_addr:
        reg_A.append(read_data(addr, i2c_register[0]))
        reg_B.append(read_data(addr, i2c_register[1]))

    for i in list_of_blinds:
        if comm == 'f':
            print(blinds[i].set_position(level))

        elif comm == 't':
            print(blinds[i].set_tilt(level))
        elif comm == 's':
            blinds[i].stop()
            print(blinds[i].position)


    if comm != 's':
        if blinds[list_of_blinds[0]].movement == 'up':
            write_data(i2c_addr[0],i2c_register[1],bld.clear_bit(reg_B[0],int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[1],bld.clear_bit(reg_B[1],int(blind2,2)))
            time.sleep(motor_delay)
            write_data(i2c_addr[0],i2c_register[0],bld.set_bit(reg_A[0],int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[0],bld.set_bit(reg_A[1],int(blind2,2)))
        elif blinds[list_of_blinds[0]].movement == 'down':
            print('down')
            write_data(i2c_addr[0],i2c_register[0],bld.clear_bit(reg_A[0],int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[0],bld.clear_bit(reg_A[1],int(blind2,2)))
            time.sleep(motor_delay)
            write_data(i2c_addr[0],i2c_register[1],bld.set_bit(reg_B,int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[1],bld.set_bit(reg_B,int(blind2,2)))
    else:
        write_data(i2c_addr[0],i2c_register[0],bld.clear_bit(reg_A[0],int(blind1,2))) #clear particular bit
        write_data(i2c_addr[1],i2c_register[0],bld.clear_bit(reg_A[1],int(blind2,2))) #clear particular bit
        write_data(i2c_addr[0],i2c_register[1],bld.clear_bit(reg_B[0],int(blind1,2)))
        write_data(i2c_addr[1],i2c_register[1],bld.clear_bit(reg_B[1],int(blind2,2)))


t1 = threading.Thread(target=blind_ctr)
#t1.start()
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(Broker, 1883, 60)
client.loop_forever()