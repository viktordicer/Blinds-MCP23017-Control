import blind as bld
import smbus
import time 
import paho.mqtt.client as mqtt
import threading


"""
All blinds connect to the expand modules MCP23017

    first device
    1 - Living          00001-000
    2 - Kitchen         00010-000
    3 - Terrace         00100-000
    4 - Bed             01000-000
    5 - Office          10000-000

    Second device
    6 - Child1          00000-001 
    7 - Child2          00000-010
    8 - Child3          00000-100
    
    MQTT command format f-100-11111-111
    f- full or t - tilt
    100 - closed is 0, opened is 100
    11111 - 5bits for first MCP23017 device with I2C address 0x20 last number is for first blind-(00001)
    111 - 3bits - for second MCP23017 device with I2C address 0x21 last number is for sixth blind-(001)
    
"""

Broker = "192.168.0.107"
sub_topic = ["sensor/blind/all"] 
pub_topic_call = "sensor/blind/calibration" 

pub_topic_position = ["sensor/blind/living-pos", "sensor/blind/kitchen-pos",
                      "sensor/blind/terrace-pos", "sensor/blind/bed-pos",
                      "sensor/blind/office-pos", "sensor/blind/child1-pos",
                      "sensor/blind/child2-pos", "sensor/blind/child3-pos"]

pub_topic_tilt = ["sensor/blind/living-tilt", "sensor/blind/kitchen-tilt",
                  "sensor/blind/terrace-tilt", "sensor/blind/bed-tilt",
                  "sensor/blind/office-tilt", "sensor/blind/child1-tilt",
                  "sensor/blind/child2-tilt", "sensor/blind/child3-tilt"]

i2c_addr = [0x20, 0x21] #I2C address for devices
i2c_register_out = [0x00, 0x01] #Set A and B to output
i2c_register = [0x12, 0x13] # 0x012 is A register and 0x13 is B register
motor_delay = 0.5 # delay between stop and command for motors

#Calibration
calibration_delay = 2#70 # all blinds go down in seconds
calibration_run = False
time_calibration = 0


#Create new objects of blinds
#dev1
living = bld.Blind(42.7, 42.3, 1.1, 0, 1)  # (full open time, full close time , tilt time, number of device, bit of blind in integer)
kitchen = bld.Blind(42.7, 42.3, 1.1, 0, 2)
terrace = bld.Blind(66.5, 65.5, 1.1, 0, 4)
bed     = bld.Blind(42.7, 42.3, 1.1, 0, 8)
office = bld.Blind(42.7, 42.3, 1.1, 0, 16)
#dev2
child1 = bld.Blind(42.7, 42.3, 1.1, 1, 1)
child2 = bld.Blind(42.7, 42.3, 1.1, 1, 2)
child3 = bld.Blind(42.7, 42.3, 1.1, 1, 4)

#MCP registers
reg_A , reg_B= [0, 0], [0, 0]

# add all blinds to the list
blinds = [living, kitchen, terrace, bed, office, child1, child2, child3]

# thread function for auto stop
def blind_ctr():
    while 1:
        time.sleep(0.05)
        global calibration_run
        if not calibration_run:
            time_now = time.time()
            read_flag = False 
            bits1 = 0
            bits2 = 0
            for i in blinds:
                if i.movement != 'stop' and not calibration_run:
                    if time_now - i.last_run > i.duration:
                        if not read_flag:
                            print('read')
                            read_registers()
                        if i.device == 0:
                            bits1 += i.bit
                        else:
                            bits2 += i.bit
                        i.stop()
                        # print(i.position)
                        # print(i.tilt_position)
                        client.publish(pub_topic_position[blinds.index(i)], i.position)
                        client.publish(pub_topic_tilt[blinds.index(i)], i.tilt_position)
                        read_flag = True
            if read_flag:
                write_data('clear',int(bits1), int(bits1), int(bits2), int(bits2))
        else:
            time_now = time.time()
            if time_now - time_calibration > calibration_delay:
                clear_register()
                calibration_run = False
                for i in blinds:
                        i.calibration_done()
                        client.publish(pub_topic_position[blinds.index(i)], i.position)
                        client.publish(pub_topic_tilt[blinds.index(i)], i.tilt_position)
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


def write_data(comm, bits_1, bits_2, bits_3, bits_4):
    if comm == 'set':
        bus.write_byte_data(i2c_addr[0],i2c_register[0],bld.set_bit(reg_A[0],bits_1))
        bus.write_byte_data(i2c_addr[0],i2c_register[1],bld.set_bit(reg_B[0],bits_2))
        bus.write_byte_data(i2c_addr[1],i2c_register[0],bld.set_bit(reg_A[1],bits_3))
        bus.write_byte_data(i2c_addr[1],i2c_register[1],bld.set_bit(reg_B[1],bits_4))
    elif comm == 'clear':
        bus.write_byte_data(i2c_addr[0],i2c_register[0],bld.clear_bit(reg_A[0],bits_1))
        bus.write_byte_data(i2c_addr[0],i2c_register[1],bld.clear_bit(reg_B[0],bits_2))
        bus.write_byte_data(i2c_addr[1],i2c_register[0],bld.clear_bit(reg_A[1],bits_3))
        bus.write_byte_data(i2c_addr[1],i2c_register[1],bld.clear_bit(reg_B[1],bits_4))


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


def get_direction(list_of_blinds):
    up1 = up2 = down1 = down2 = 0

    for i in list_of_blinds:
        if i < 5:
            if blinds[i].movement == 'up':
                up1 += blinds[i].bit
            else:
                down1 += blinds[i].bit
        else:
            if blinds[i].movement == 'up':
                up2 += blinds[i].bit
            else:
                down2 += blinds[i].bit                
    return up1, down1, up2, down2


def read_registers():
    global reg_A
    global reg_B
    for i in range(len(i2c_addr)):
        reg_A[i] = (read_data(i2c_addr[i], i2c_register[0]))
        reg_B[i] = (read_data(i2c_addr[i], i2c_register[1]))


def send_command(message):
    comm, level, blind1, blind2 = bld.decode_command(message)
    list_of_blinds = bld.find_index(blind1,blind2)
#read all registers to list
    read_registers()

    for i in list_of_blinds:
        if comm == 'f':
            blinds[i].stop()
            blinds[i].set_position(level)
        elif comm == 't':
            blinds[i].stop()
            blinds[i].set_tilt(level)
        elif comm == 's':
            blinds[i].stop()
            print(blinds[i].position)
            client.publish(pub_topic_position[i], blinds[i].position)
            client.publish(pub_topic_tilt[i], blinds[i].tilt_position)

    up1, down1, up2, down2 = get_direction(list_of_blinds)

    if comm != 's' :
        write_data('clear', down1, up1, down2, up2)
        time.sleep(motor_delay)
        read_registers()
        write_data('set', up1, down1, up2, down2)
    else:
        write_data('clear', int(blind1,2), int(blind1,2), int(blind2,2), int(blind2,2))


def not_calib_msg():
    client.publish(pub_topic_call, "NOT calibrated")
    for i in range(len(pub_topic_position)):
        client.publish(pub_topic_position[i], 'error') 
        client.publish(pub_topic_tilt[i], 'error') 


def on_connect(client, rc):
    print("Connected with result code "+str(rc))
    for i in sub_topic:
        client.subscribe(i)


def on_message(client, msg):
    message = str(msg.payload.decode("utf-8"))
    print("message:" + message)
    global calibration_run
    if message == 'calibrate':
        calibration()
    else:
        if calibration_run and message[:1] == 's':
            clear_register()
            calibration_run = False
            client.publish(pub_topic_call,'stopped - error')
        else:
            send_command(message)


# create object smbus
bus = smbus.SMBus(1)

# MCP23017 - SET REGISTER AS OUTPUTS AND CLEAR ALL REGISTERS
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
