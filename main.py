import blind as bld
import smbus
import time 
import paho.mqtt.client as mqtt
import threading
from array import *


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
    
    MQTT command format f-100-11111-111
    f- full or t - tilt
    100 - closed is 0, opened is 100
    11111 - 5bits for first MCP23017 device with I2C address 0x20 last number is for first blind-(00001)
    111 - 3bits - for second MCP23017 device with I2C address 0x21 last number is for sixth blind-(001)
    
"""

Broker = "192.168.0.122"
sub_topic = ["sensor/blind/all"] 
pub_topic_call = "sensor/blind/calibration" 

pub_topic_position = ["sensor/blind/living-pos", "sensor/blind/kitchen-pos", "sensor/blind/terrace-pos", "sensor/blind/bed-pos",
                      "sensor/blind/office-pos", "sensor/blind/child1-pos", "sensor/blind/child2-pos", "sensor/blind/child3-pos"]
pub_topic_tilt = ["sensor/blind/living-tilt", "sensor/blind/kitchen-tilt", "sensor/blind/terrace-tilt", "sensor/blind/bed-tilt",
                      "sensor/blind/office-tilt", "sensor/blind/child1-tilt", "sensor/blind/child2-tilt", "sensor/blind/child3-tilt"]

i2c_addr = [0x20, 0x21] #I2C address for devices
i2c_register_out = [0x00, 0x01] #Set A and B to output
i2c_register = [0x12, 0x13] # 0x012 is A register and 0x13 is B register
motor_delay = 0.5 # delay between stop and command for motors

#Calibration
calibration_delay = 2 # all blinds go down in seconds
calibration_run = False
time_calibration = 0

blinds = [] # list of objects all blinds
#Create new objects of blinds
#dev1
living = bld.Blind(42.7,42.3,1.2,0,1)  # (full open time, full close time , tilt time, number of device, bit of blind in integer)
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


# thread function for auto stop
def blind_ctr():
    while 1:
        global calibration_run
        if calibration_run == False:
            time_now = time.time()
            for i in blinds:
                if i.movement != 'stop' and calibration_run == False:
                    if time_now - i.last_run > i.duration:
                        reg_A = read_data(i2c_addr[i.device], i2c_register[0]) #
                        reg_B = read_data(i2c_addr[i.device], i2c_register[1])
                        write_data(i2c_addr[i.device],i2c_register[0],bld.clear_bit(reg_A,int(i.bit))) #clear particular bit
                        write_data(i2c_addr[i.device],i2c_register[1],bld.clear_bit(reg_B,int(i.bit)))
                        i.stop()
                        print(i.position)
                        client.publish(pub_topic_position[blinds.index(i)], i.position)
                        client.publish(pub_topic_tilt[blinds.index(i)], i.tilt_position)
        else:
            time_now = time.time()
            if time_now - time_calibration > calibration_delay:
                clear_register()
                calibration_run = False
                for i in range(len(pub_topic_position)):
                    client.publish(pub_topic_position[i], '0') 
                    client.publish(pub_topic_tilt[i], '0') 
                client.publish(pub_topic_call, 'done')

def set_reg_as_output():
    for dev in i2c_addr:
        for reg_out in i2c_register_out:
            bus.write_byte_data(dev,reg_out,0x00)

def clear_register():
    for dev in i2c_addr:
        for reg in i2c_register:
            bus.write_byte_data(dev,reg,0) # Clear register A and B

#read data from register
def read_data(device,register):
    return bus.read_byte_data(device, register)

#write data to the particular bit
def write_data(device,register,bit):
    bus.write_byte_data(device,register,bit)


def calibration():
    global time_calibration
    global calibration_run
    print('calibration')
    clear_register()
    time.sleep(motor_delay)
    for dev in i2c_addr:
        bus.write_byte_data(dev,i2c_register[1],255)
    time_calibration = time.time()
    calibration_run = True
    client.publish(pub_topic_call, 'running')


def send_command(message):
    comm, level, blind1, blind2 = bld.decode_command(message)
    list_of_blinds = bld.find_index(blind1+blind2)
    
    reg_A = []
    reg_B=[] #read all registers to list
    for addr in i2c_addr:
        reg_A.append(read_data(addr, i2c_register[0]))
        reg_B.append(read_data(addr, i2c_register[1]))

    for i in list_of_blinds:
        if comm == 'f':
            blinds[i].set_position(level)
        elif comm == 't':
            blinds[i].set_tilt(level)
        elif comm == 's':
            blinds[i].stop()
            print(blinds[i].position)
            client.publish(pub_topic_position[i], blinds[i].position)
            client.publish(pub_topic_tilt[i], blinds[i].tilt_position)

    if comm != 's' :
        if blinds[list_of_blinds[0]].movement == 'up':
            write_data(i2c_addr[0],i2c_register[1],bld.clear_bit(reg_B[0],int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[1],bld.clear_bit(reg_B[1],int(blind2,2)))
            time.sleep(motor_delay)
            write_data(i2c_addr[0],i2c_register[0],bld.set_bit(reg_A[0],int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[0],bld.set_bit(reg_A[1],int(blind2,2)))
        elif blinds[list_of_blinds[0]].movement == 'down':
            write_data(i2c_addr[0],i2c_register[0],bld.clear_bit(reg_A[0],int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[0],bld.clear_bit(reg_A[1],int(blind2,2)))
            time.sleep(motor_delay)
            write_data(i2c_addr[0],i2c_register[1],bld.set_bit(reg_B[0],int(blind1,2)))
            write_data(i2c_addr[1],i2c_register[1],bld.set_bit(reg_B[1],int(blind2,2)))
    else:
        write_data(i2c_addr[0],i2c_register[0],bld.clear_bit(reg_A[0],int(blind1,2))) 
        write_data(i2c_addr[1],i2c_register[0],bld.clear_bit(reg_A[1],int(blind2,2))) 
        write_data(i2c_addr[0],i2c_register[1],bld.clear_bit(reg_B[0],int(blind1,2)))
        write_data(i2c_addr[1],i2c_register[1],bld.clear_bit(reg_B[1],int(blind2,2)))

def not_calib_msg():
    client.publish(pub_topic_call, "NOT calibrated")
    for i in range(len(pub_topic_position)):
        client.publish(pub_topic_position[i], 'error') 
        client.publish(pub_topic_tilt[i], 'error') 

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    for i in sub_topic:
        client.subscribe(i)


def on_message(client, userdata, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    global calibration_run
    if message == 'calibrate':
        calibration()
    else:
        if calibration_run == True and message[:1] == 's':
            clear_register()
            calibration_run = False
            client.publish(pub_topic_call,'stopped - error')
        else:
            send_command(message)
#create object smbus
bus = smbus.SMBus(1)

##	MCP23017 - SET REGISTER AS OUTPUTS AND CLEAR ALL REGISTERS
set_reg_as_output()
clear_register()


# run auto stop thread
t1 = threading.Thread(target=blind_ctr)
t1.start()

#mqtt connection
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(username="viktor", password="viktor")
client.connect(Broker, 1883, 60)
not_calib_msg()
client.loop_forever()
