
import sys
import time
import os
import can
import logging
from datetime import datetime
import asyncio

from numpy import *
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtWidgets, QtCore
from threading import Thread

import pigpio

#logging.basicConfig(level=logging.DEBUG)

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

        #print(f"arg_id = {self.arb_id}, dlc = {self.dlc}, angle = {self.angle}, speed = {self.speed}, temp = {self.temperature}")

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
        #print(f"found arb id table = {resp_can_info}")
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

        #print(f"output_data = {output_data}")
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

        #self.read_can_thread()
        #self.start()

    def setUpdateTimeIntervalMS(self,interval_ms):
        self.time_interval_ms = interval_ms

    # Read in data using a thread
    def read_can_thread(self):
        self.current_position_value = 0
        self.old_current_plot_value = 0
        self.position_update_thread = Thread(target=self.read_can, args=())
        self.position_update_thread.daemon = True
        self.position_update_thread.start()

    def read_can(self):
        while 1:
            message = self.recv(time_interval_ms)
   
            if message is None:
                print('Timeout occurred, no message received.')
            else:
                #print (message)
                #print(f"formated time stamp = {datetime.fromtimestamp(message.timestamp)}")
                self.can_handler_obj.on_msg_received(message,self)
            self.current_plot_value = self.can_handler_obj.get_plot_data()
            self.old_current_plot_value = self.current_plot_value

    def get_plot_data(self):
        return self.current_plot_value

    def start(self):
        #os.system(f'sudo ip link set {self.CAN_INF} type can bitrate {self.CAN_BIT_RATE}')
        #os.system(f'sudo ifconfig {self.CAN_INF} up')

        self.loop = asyncio.get_event_loop()
        self.notifier = can.Notifier(self.bus,[self.on_message_received],None,self.loop)

    def start_can_read_thread(self):
        self.read_can_thread()
    
    def recv(self,timeout_ms):
        msg = self.bus.recv(timeout_ms)
        return msg
    # message.arbitration_id : int
    # message.data : bytearray
    # message.timestamp : float
    # message.dlc : int
    def on_message_received(self,msg):
        #print (msg)
        #print(f"formated time stamp = {datetime.fromtimestamp(msg.timestamp)}")
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

        #self.pid_out_max = 300
        #self.pid_out_min = -300

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
        #self.pid_ref = self.value_wrap(ref_in,round_value)
        self.pid_fdb = fdb_in
        #self.pid_fdb = self.value_wrap(fdb_in,round_value)

        # Save previous one
        self.pid_error_1 = self.pid_error_0

        # Update latest one
        #self.pid_error_0 = self.pid_ref - self.pid_fdb
        self.pid_error_0 = self.pid_ref - self.pid_fdb
        self.pid_error_0 = self.value_wrap(self.pid_error_0,round_value)
        #self. = self.value_wrap(fdb_in,round_value)


        #print(f"original error 0 value = {self.pid_error_0}")
        #if self.pid_error_0 < 0:
        #    is_negative = True
        #else:   
        #    is_negative = False

        #if not round_value == 0:
        #    temp = self.pid_error_0 % round_value
        #    if is_negative:
        #        temp = temp - round_value
            #if temp > (round_value//2):
            #    temp = temp - round_value
        #    self.pid_error_0 = temp
        #print(f"changed error 0 value = {self.pid_error_0}")

        #self.pid_error_0 =  self.pid_error_0 * changed_round_value // round_value
                

        self.pid_p_out = self.Kp * self.pid_error_0
        self.pid_i_out = self.pid_i_out + self.Ki * self.pid_error_0
        self.pid_d_out = self.Kd * ( self.pid_error_0 - self.pid_error_1)

        if self.pid_i_out > self.pid_out_max:
            self.pid_i_out = self.pid_out_max

        if self.pid_i_out < self.pid_out_min:
            self.pid_i_out = self.pid_out_min

        self.pid_output = int(self.pid_p_out + self.pid_i_out + self.pid_d_out)
        #print(f" pid_output = {self.pid_output} , p_out = {self.pid_p_out} , i_out = {self.pid_i_out} , d_out = {self.pid_d_out}")

        if self.pid_output > self.output_max:
            self.pid_output = self.output_max

        if self.pid_output < (-1 * self.output_max):
            self.pid_output = (-1 * self.output_max)

        return self.pid_output

    def value_wrap(self,raw_input,round_value):
        if round_value == 0:
            return raw_input
        temp = raw_input % round_value
        if temp > (round_value//2):
            temp = temp - round_value
        return temp

class CAN_HANDLER():
    def __init__(self):
        self.angle_degree_max = 360
        self.angle_degree_min = 0

        self.angle_value_max = 8191
        self.angle_value_min = 0

        self.angle_degree_round = self.angle_degree_max
        self.angle_value_round = int(self.angle_degree_max * self.angle_value_max / self.angle_degree_round)

        self.speed_rpm_max = 60
        self.speed_rpm_min = -60

    def set_time_interval_ms(self,time_interval_ms_input):
        self.time_interval_ms = time_interval_ms_input

    def get_time_interval_ms(self):
        return self.time_interval_ms

    def set_inner_pid(self,pid_input):
        self.inner_pid = pid_input

    def set_outer_pid(self,pid_output):
        self.outer_pid = pid_output

    def set_gpio_log(self,gpio_log):
        self.gpio_log_obj = gpio_log

    def set_plot_handler(self,plot_widget):
        self.plot_obj = plot_widget

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

    def set_plot_data(self,plot_data_input):
        self.plot_data = plot_data_input

    def get_plot_data(self):
        return self.plot_data

    def on_msg_received(self,msg,can_manager):
        #self.gpio_log_obj.set_TRIGGERED_STATE()

        # get from raw can msg
        msgMotor = CanMotorMessage()

        msgMotor.unpack_in_raw_can_frame(msg)
        fdb_angle = msgMotor.get_angle()
        self.set_plot_data(fdb_angle)

        fdb_speed = msgMotor.get_speed()
        arb_id = msgMotor.get_can_arb_id()
        if not msgMotor.is_ID_valid():
            print(f"invalid arb id")
            return

        # --------------------for double loop
        target_speed = self.outer_pid.pid_calc(self.target_angle_value,fdb_angle,self.angle_value_round)
        #print(f"pid outer calc result = {target_speed} (rpm) , fdb_angle = {fdb_angle} , target angle = {self.target_angle_value} ")
        target_voltage = self.inner_pid.pid_calc(target_speed,fdb_speed,0)
        #target_voltage = self.inner_pid.pid_calc(target_speed*60,fdb_speed*60,0)
        #print(f"pid inner calc result = {target_voltage} (v) , fdb speed = {fdb_speed} rpm")

        # --------------------for single loop
        #target_voltage = self.outer_pid.pid_calc(self.target_angle_value,fdb_angle,self.angle_value_round)
        #print(f"pid outer calc result = {target_voltage} (voltage) , fdb_angle = {fdb_angle} , target angle = {self.target_angle_value} ")

        mortor_msg_output = CanMotorMessage(arb_id)
        mortor_msg_output.set_voltage(target_voltage)
        mortor_msg_output.build_out_raw_can_frame()
        can_manager.send_to_motor(mortor_msg_output.get_raw_can_msg())

        #self.gpio_log_obj.set_INIT_STATE()
        #self.plot_obj.plot_pid_updater(self.target_angle_value,fdb_angle,target_voltage)

class ScrollingPlot(QtGui.QWidget):
    def __init__(self, parent=None):
        super(ScrollingPlot, self).__init__(parent)

        # Desired Time Interval = 1 / self.FREQUENCY
        # USE FOR TIME.SLEEP (s)
        #self.UPDATE_INTERVAL_SEC = .004
        #self.UPDATE_INTERVAL_SEC = .01
        self.UPDATE_INTERVAL_SEC = 1

        # Frequency to update plot (ms)
        # USE FOR TIMER.TIMER (ms)
        #self.TIMER_FREQUENCY = self.UPDATE_INTERVAL_SEC * 1000
        self.time_interval_ms = self.UPDATE_INTERVAL_SEC * 1000

        # Set X Axis range. If desired is [-10,0] then set LEFT_X = -10 and RIGHT_X = 0
        self.LEFT_X = -100
        self.RIGHT_X = 0
        self.X_Axis = np.arange(self.LEFT_X, self.RIGHT_X, self.UPDATE_INTERVAL_SEC)
        self.buffer = int((abs(self.LEFT_X) + abs(self.RIGHT_X))/self.UPDATE_INTERVAL_SEC)
        self.data = [] 

        # Create Plot Widget 
        self.scrolling_plot_widget = pg.PlotWidget()

        # Enable/disable plot squeeze (Fixed axis movement)
        self.scrolling_plot_widget.plotItem.setMouseEnabled(x=False, y=False)
        self.scrolling_plot_widget.setXRange(self.LEFT_X, self.RIGHT_X)
        self.scrolling_plot_widget.setTitle('Scrolling Plot Example')
        self.scrolling_plot_widget.setLabel('left', 'Value')
        self.scrolling_plot_widget.setLabel('bottom', 'Time (s)')

        self.scrolling_plot = self.scrolling_plot_widget.plot()
        self.scrolling_plot.setPen(197,235,255)

        self.layout = QtGui.QGridLayout()
        self.layout.addWidget(self.scrolling_plot_widget)

        #self.read_position_thread()
        #self.start_plot()

    def setUpdateTimeIntervalMS(self,interval_ms):
        self.time_interval_ms = interval_ms

    def set_can_manager(self,can_manager):
        self.can_manager = can_manager

    # Update plot
    def start_plot(self):
        self.position_update_timer = QtCore.QTimer()
        #self.position_update_timer.timeout.connect(self.plot_updater)
        self.position_update_timer.timeout.connect(self.plot_pid_updater)
        self.position_update_timer.start(self.get_scrolling_plot_timer_frequency())


    # Read in data using a thread
    def read_position_thread(self):
        self.current_position_value = 0
        self.old_current_position_value = 0
        self.position_update_thread = Thread(target=self.read_position, args=())
        self.position_update_thread.daemon = True
        self.position_update_thread.start()

    def read_position(self):
        frequency = self.get_scrolling_plot_frequency()
        while True:
            try:
                # Add data
                self.current_position_value = random.randint(1,101) 
                self.old_current_position_value = self.current_position_value
                time.sleep(frequency)
            except:
                self.current_position_value = self.old_current_position_value

    def plot_pid_updater(self):
        #print("[plot_pid_updater] Enter")
        try:
            self.dataPoint = self.can_manager.get_plot_data()
            #print(f"[plot_pid_updater] self.dataPoint = {self.dataPoint}")
        except:
            print("[plot_pid_updater] Exception")
            return

        if len(self.data) >= self.buffer:
            del self.data[:1]
        self.data.append(self.dataPoint)
        self.scrolling_plot.setData(self.X_Axis[len(self.X_Axis) - len(self.data):], self.data)

    def plot_updater(self):
        self.dataPoint = float(self.current_position_value)

        if len(self.data) >= self.buffer:
            del self.data[:1]
        self.data.append(self.dataPoint)
        self.scrolling_plot.setData(self.X_Axis[len(self.X_Axis) - len(self.data):], self.data)

    def clear_scrolling_plot(self):
        self.data[:] = []

    def get_scrolling_plot_frequency(self):
        return self.UPDATE_INTERVAL_SEC

    def get_scrolling_plot_timer_frequency(self):
        #return self.TIMER_FREQUENCY
        return self.time_interval_ms

    def get_scrolling_plot_layout(self):
        return self.layout

    def get_current_position_value(self):
        return self.current_position_value

    def get_scrolling_plot_widget(self):
        return self.scrolling_plot_widget

class LogicAnalyzerLog():
    #def __init__(self,power_connection_input="NO",gpio_pin_input=12):
    def __init__(self,gpio_pin_input=12):
        self.pigpio = pigpio.pi()
        #self.power_connection = power_connection_input

        # NO : Normal Connection , NC : Normal Open
        #self.relay_power_connection_map = {"NC":0,"NO":1}
        #self.relay_state_map = {"INIT_STATE":1,"TRIGGERED_STATE":0}
        self.relay_state_map = {"INIT_STATE":0,"TRIGGERED_STATE":1}
        #self.is_diff_default_power_setting = self.relay_power_connection_map[self.power_connection]
        self.gpio_pin = gpio_pin_input

        self.set_INIT_STATE()

        # Keep Normal Connect / Disconnect
    def set_INIT_STATE(self):
        #value = bool(self.relay_state_map["INIT_STATE"]) ^ bool(self.is_diff_default_power_setting)
        value = bool(self.relay_state_map["INIT_STATE"])
        self.pigpio.write(self.gpio_pin,int(value))

    def set_TRIGGERED_STATE(self):
        #value = bool(self.relay_state_map["TRIGGERED_STATE"]) ^ bool(self.is_diff_default_power_setting)
        value = bool(self.relay_state_map["TRIGGERED_STATE"])
        self.pigpio.write(self.gpio_pin,int(value))


if __name__ == '__main__':

    os.system('sudo ip link set can0 up type can bitrate 1000000')
    os.system('sudo ifconfig can0 txqueuelen 2000') # Enable can0

    # -------------------For single loop best
    # Currently best
    #pid_position = PID_ONE_CLOSE_LOOP(74,1,0,0,8191,-3000,3000)

    # -------------------For double loop best
    # Currently best
    #pid_position = PID_ONE_CLOSE_LOOP(1,0,0,0,8191,-300,300)
    #pid_rotation = PID_ONE_CLOSE_LOOP(40,1,0,-300,300,-3000,3000)

    # Currently best
    pid_position = PID_ONE_CLOSE_LOOP(1,0,0,0,8191,-10,10)
    pid_rotation = PID_ONE_CLOSE_LOOP(85,0,25,-10,10,-3000,3000)

    #pid_position = PID_ONE_CLOSE_LOOP(1,0,0,0,8191,-150,150)
    #pid_rotation = PID_ONE_CLOSE_LOOP(85,0,25,-150,150,-3000,3000)

    can_handler = CAN_HANDLER()
    can_handler.set_inner_pid(pid_rotation)
    can_handler.set_outer_pid(pid_position)

    target_angle_degree = 180
    time_interval_ms = 10

    can_handler.set_target_angle(target_angle_degree)
    can_handler.set_time_interval_ms(time_interval_ms)
    #can_handler.set_target_speed(target_speed_rpm)

    can_manager = CanManager(can_handler)
    can_manager.setUpdateTimeIntervalMS(time_interval_ms)
    #can_manager.start()




    # Create main application window
    app = QtGui.QApplication([])
    app.setStyle(QtGui.QStyleFactory.create("Cleanlooks"))
    mw = QtGui.QMainWindow()
    mw.setWindowTitle('Scrolling Plot Example')

    # Create scrolling plot
    # -------------------------------------------------QT
    # 10 ms
    
    plot_widget = ScrollingPlot()
    plot_widget.setUpdateTimeIntervalMS(1000) #1000 ms
    #can_handler.set_plot_handler(plot_widget)
    gpio_log = LogicAnalyzerLog()
    can_handler.set_gpio_log(gpio_log)
    # end -------------------------------------------------QT

    # Create and set widget layout
    # Main widget container
    cw = QtGui.QWidget()
    ml = QtGui.QGridLayout()
    cw.setLayout(ml)
    mw.setCentralWidget(cw)

    #print("test(0)")
    # Can use either to add plot to main layout
    #ml.addWidget(plot_widget.get_scrolling_plot_widget(),0,0)
    ml.addLayout(plot_widget.get_scrolling_plot_layout(),0,0)
    mw.show()
    #print("test(1)")

    plot_widget.set_can_manager(can_manager)
    plot_widget.start_plot()
    can_manager.start_can_read_thread()

    # Start Qt event loop unless running in interactive mode or using pyside
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

    #print("test(2)")


    os.system('sudo ifconfig can0 down') # Enable can0

    #while 1:

        #message = bus.recv(10.0)
    #    message = can_manager.recv(time_interval_ms)
   
    #    if message is None:
    #        print('Timeout occurred, no message received.')
    #    else:
            #print (message)
            #print(f"formated time stamp = {datetime.fromtimestamp(message.timestamp)}")
    #        can_handler.on_msg_received(message,can_manager)
        #time.sleep(1)

        #app.processEvents()


