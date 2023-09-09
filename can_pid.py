
import time
import os
import can
import logging
from datetime import datetime
import asyncio

logging.basicConfig(level=logging.DEBUG)

#os.system('sudo ip link set can0 type can bitrate 1000000')
#os.system('sudo ifconfig can0 up') # Enable can0


class CanMotorMessage():
    def __init__(self,arb_id_in=0x1FF):
        self.DIR_IN = 0
        self.DIR_OUT = 1

        #self.dir_out_in = dir_out_in

        self.arb_id = arb_id_in
        self.dlc = 0

        # Unpack in raw frame
        self.can_sentout_voltage = 0

        self.out_info = {
                                0x205:(0x1FF,0,1,8,1),
                                0x206:(0x1FF,2,3,8,2),
                                0x207:(0x1FF,4,5,8,3),
                                0x208:(0x1FF,6,7,8,4),

                                0x209:(0x2FF,0,1,8,5),
                                0x20A:(0x2FF,2,3,8,6),
                                0x20B:(0x2FF,4,5,8,7),
                            }
        self.default_out_can_frame = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

    # Unpack in raw frame
    def unpack_in_raw_can_frame(self,can_msg):
        self.arb_id = can_msg.arbitration_id
        self.dlc = can_msg.dlc
        self.raw_data = can_msg.data

        #raw = self.raw_data[0:2]
        #self.angle = int.from_bytes(raw, 'big',signed = True) 
        if (0x80 & self.raw_data[0]) != 0:
            temp = (((0xFF>>1) & self.raw_data[0]) << 8) | self.raw_data[1]
            self.angle = -(0x00008000 -  temp)
        else:
            self.angle = self.raw_data[0] << 8 | self.raw_data[1]
        #raw = self.raw_data[2:4]
        #self.speed = int.from_bytes(raw, 'big',signed = True) 
        if (0x80 & self.raw_data[2]) != 0:
            temp = (((0xFF>>1) & self.raw_data[2]) << 8) | self.raw_data[3]
            self.speed = -(0x00008000 -  temp)
        else:
            self.speed = self.raw_data[2] << 8 | self.raw_data[3]
        self.temperature = self.raw_data[6]

        print(f"arg_id = {self.arb_id}, dlc = {self.dlc}, angle = {self.angle}, speed = {self.speed}, temp = {self.temperature}")

    def get_angle(self):
        return self.angle

    def get_speed(self):
        return self.speed

    def get_can_arb_id(self):
        return self.arb_id


    # Build out raw frame
    def set_voltage(self,voltage_input):
        self.can_sentout_voltage = voltage_input

    def is_ID_valid(self):
        if self.arb_id in self.out_info.keys():
            return True
        return False

    def build_out_raw_can_frame(self):
        voltage = self.can_sentout_voltage

        resp_can_info = self.out_info[self.arb_id]
        print(f"found arb id table = {resp_can_info}")
        high_index = resp_can_info[1]
        low_index = resp_can_info[2]

        if voltage < 0:
            raw = 0x00008000 - abs(voltage)
            raw = raw | 0x00008000
        else:
            raw = voltage
        output_data = self.default_out_can_frame
        #output_data[high_index] = (0xff00 & voltage) >> 8
        #output_data[low_index] = 0x00ff & voltage
        output_data[high_index] = (0xff00 & raw) >> 8
        output_data[low_index] = 0x00ff & raw

        print(f"output_data = {output_data}")
        self.out_msg = can.Message(arbitration_id=resp_can_info[0], dlc=resp_can_info[3], is_extended_id=False, data=output_data)

    def get_raw_can_msg(self):
        return self.out_msg

class CanManager(can.Listener):
    def __init__(self,can_handler_obj):
        self.CAN_INF = "can0"
        self.CAN_BUS_TYPE = "socketcan"
        self.CAN_BIT_RATE = 1000000

        self.bus = can.Bus(channel = self.CAN_INF, bustype = self.CAN_BUS_TYPE)# socketcan_native


        # Register self
        self.can_handler_obj = can_handler_obj

    def start(self):
        #os.system(f'sudo ip link set {self.CAN_INF} type can bitrate {self.CAN_BIT_RATE}')
        #os.system(f'sudo ifconfig {self.CAN_INF} up')

        self.loop = asyncio.get_event_loop()
        self.notifier = can.Notifier(self.bus,[self.on_message_received],None,self.loop)
    
    def recv(self,timeout_ms):
        msg = self.bus.recv(timeout_ms)
        return msg
    # message.arbitration_id : int
    # message.data : bytearray
    # message.timestamp : float
    # message.dlc : int
    def on_message_received(self,msg):
        print (msg)
        print(f"formated time stamp = {datetime.fromtimestamp(msg.timestamp)}")
        self.can_handler_obj.on_msg_received(msg,self)

    def on_error(self,exec):
        print("can bus error")

    def send_to_motor(self,msg):
        self.bus.send(msg)


class PID_ONE_CLOSE_LOOP():
    def __init__(self,kp_input,ki_input,kd_input,input_min_in,input_max_in,output_min_in,output_max_in):
        self.Kp = kp_input
        self.Ki = ki_input
        self.Kd = kd_input

        self.pid_out_max = 30000
        self.pid_out_min = -30000

        self.input_min = input_min_in
        self.input_max = input_max_in
        self.output_min = output_min_in
        self.output_max = output_max_in

        self.pid_ref = 0;
        self.pid_fdb = 0;

        self.pid_error_0 = 0
        self.pid_error_1 = 0

        self.pid_p_out = 0
        self.pid_i_out = 0
        self.pid_d_out = 0

        self.pid_output = 0

    def pid_calc(self,ref_in,fdb_in,round_value):
        if ref_in < self.input_min:
            ref_in = self.input_min

        if ref_in > self.input_max:
            ref_in = self.input_max

        self.pid_ref = ref_in
        self.pid_fdb = fdb_in

        # Save previous one
        self.pid_error_1 = self.pid_error_0

        # Update latest one
        self.pid_error_0 = self.pid_ref - self.pid_fdb
        if not round_value == 0:
            temp = self.pid_error_0 % round_value
            if temp > (round_value//2):
                temp = temp - round_value
            self.pid_error_0 = temp
                

        self.pid_p_out = self.Kp * self.pid_error_0
        self.pid_i_out = self.pid_i_out + self.Ki * self.pid_error_0
        self.pid_d_out = self.Kd * ( self.pid_error_0 - self.pid_error_1)

        if self.pid_i_out > self.pid_out_max:
            self.pid_i_out = self.pid_out_max

        if self.pid_i_out < self.pid_out_min:
            self.pid_i_out = self.pid_out_min

        self.pid_output = int(self.pid_p_out + self.pid_i_out + self.pid_d_out)
        print(f" pid_output = {self.pid_output} , p_out = {self.pid_p_out} , i_out = {self.pid_i_out} , d_out = {self.pid_d_out}")

        if self.pid_output > self.output_max:
            self.pid_output = self.output_max

        if self.pid_output < (-1 * self.output_max):
            self.pid_output = (-1 * self.output_max)

        return self.pid_output

class CAN_HANDLER():
    def __init__(self):
        self.angle_degree_max = 360
        self.angle_degree_min = 0

        self.angle_value_max = 8191
        self.angle_value_min = 0

        self.angle_degree_round = 360
        self.angle_value_round = int(self.angle_degree_max * self.angle_value_max / 360)

        self.speed_rpm_max = 60
        self.speed_rpm_min = -60

    def set_inner_pid(self,pid_input):
        self.inner_pid = pid_input

    def set_outer_pid(self,pid_output):
        self.outer_pid = pid_output

    def set_can_manager(self,can_manager_in):
        self.can_manager = can_manager_in

    def set_target_angle(self,target_angle_degree_in):
        if target_angle_degree_in > self.angle_degree_max:
            target_angle_degree_in = self.angle_degree_max
        if target_angle_degree_in < self.angle_degree_min:
            target_angle_degree_in = self.angle_degree_min
        self.target_angle_degree = target_angle_degree_in
        self.target_angle_value = int(self.target_angle_degree * self.angle_value_max / self.angle_degree_max)

    def set_motor_id(self,motor_id_in):
        self.motor_id = motor_id_in

    def on_msg_received(self,msg,can_manager):
        # get from raw can msg
        msgMotor = CanMotorMessage()

        msgMotor.unpack_in_raw_can_frame(msg)
        fdb_angle = msgMotor.get_angle()
        fdb_speed = msgMotor.get_speed()
        arb_id = msgMotor.get_can_arb_id()
        if not msgMotor.is_ID_valid():
            print(f"invalid arb id")
            return

        #target_speed = self.outer_pid.pid_calc(self.target_angle_value,fdb_angle,self.angle_value_round)
        #fdb_angle_degree = int(fdb_angle * self.angle_degree_max / self.angle_value_max)
        #target_speed = self.outer_pid.pid_calc(self.target_angle_degree,fdb_angle_degree,self.angle_degree_round)
        #print(f"pid outer calc result = {target_speed} (rpm) , fdb_angle = {fdb_angle} , target angle = {self.target_angle_value} ")
        #target_voltage = self.inner_pid.pid_calc(target_speed,fdb_speed,0)
        #print(f"pid inner calc result = {target_voltage} (v) , fdb speed = {fdb_speed} rpm")

        target_voltage = self.outer_pid.pid_calc(self.target_angle_value,fdb_angle,self.angle_value_round)
        print(f"pid outer calc result = {target_voltage} (voltage) , fdb_angle = {fdb_angle} , target angle = {self.target_angle_value} ")

        mortor_msg_output = CanMotorMessage(arb_id)
        mortor_msg_output.set_voltage(target_voltage)
        mortor_msg_output.build_out_raw_can_frame()
        can_manager.send_to_motor(mortor_msg_output.get_raw_can_msg())

if __name__ == '__main__':

    #correct one(single PID)
    #pid_position = PID_ONE_CLOSE_LOOP(64,2,0,0,8191,-8000,8000)

    #pid_position = PID_ONE_CLOSE_LOOP(30,0,1,0,8191,-8000,8000)
    pid_position = PID_ONE_CLOSE_LOOP(74,1,0,0,8191,-3000,3000)
    # Position loop outenr
    #pid_position = PID_ONE_CLOSE_LOOP(80,0,0,0,8191,-300,300)
    #pid_position = PID_ONE_CLOSE_LOOP(10,1,0,0,8191,-30000,30000)
    #pid_position = PID_ONE_CLOSE_LOOP(64,100,0,0,8191,-30000,30000)

    # Rotation loop inner
    #pid_rotation = PID_ONE_CLOSE_LOOP(64,100,0,-60,60,-30000,30000)
    pid_rotation = PID_ONE_CLOSE_LOOP(40,3,0,-300,300,-30000,30000)
    #pid_rotation = PID_ONE_CLOSE_LOOP(1,100,0,0,60,-30000,30000)

    can_handler = CAN_HANDLER()
    can_handler.set_inner_pid(pid_rotation)
    can_handler.set_outer_pid(pid_position)

    target_angle_degree = 120
    can_handler.set_target_angle(target_angle_degree)
    #can_handler.set_target_speed(target_speed_rpm)

    can_manager = CanManager(can_handler)
    #can_manager.start()


    while 1:

        #message = bus.recv(10.0)
        message = can_manager.recv(10.0)
   
        if message is None:
            print('Timeout occurred, no message received.')
        else:
            #print (message)
            #print(f"formated time stamp = {datetime.fromtimestamp(message.timestamp)}")
            can_handler.on_msg_received(message,can_manager)
        #time.sleep(1)


