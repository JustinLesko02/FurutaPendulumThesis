#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 11 21:12:17 2026

@author: justinlesko
More buttons for controls

"""
import sysidentpy
import pandas as pd
import numpy as np
import torch
import matplotlib
import scipy
from sysidentpy.utils.generate_data import get_siso_data
from sysidentpy.model_structure_selection import FROLS
from sysidentpy.basis_function import Polynomial
from sysidentpy.parameter_estimation import LeastSquares
from sysidentpy.utils.display_results import results
import serial
import re
import time
import matplotlib.figure as figure
import math
import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *

import numpy as np
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
from matplotlib.figure import Figure


class FurutaTester:
    def __init__(self):
        """
        Initializes the tkinter interface for testing

        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        # This section sets up the necessary data processing / graphing variables such as variable lists for plotting and calls all of the functions for setting up the different parts of the tester

        self.root = tk.Tk() #: Root for the tkinter interface
        self.i = 0 #: Index for the 
        self.update_graph_flag = 1 #: Flag for whether to update the graph
        self.calibration_flag = 0 #: Indicates whether the model is calibrated
        self.data_stripped = [] #: List for the stripped data
        print("length:", len(self.data_stripped))

        self.serial_process_timing = 0
        self.plot_timing = 0
        self.data_array = np.zeros(6,1)
        self.motor_angle = [] #: List for the motor angle
        self.motor_speed = [] #: List for the motor speed
        self.pendulum_angle = [] #: List for the pend angle
        self.pendulum_speed = [] #: List for the pend speed
        self.motor_effort = [] #: List for the motor effort
        self.t = [] #: List for the time

        self.pendulum_encoder_ticks = 4096 #: Pendulum encoder ticks per revolution
        self.motor_encoder_ticks = 8192 #: Motor encoder ticks per revolution
        self.data_raw = "" #: String for the raw input data

        # This section sets up the actual ttk frame, including all necessary sections, style, and titles. Lastly, the two main loops for graphing and reading are called. 
        self.style = ttk.Style(theme = 'darkly') #: overall ttk frame style
        self.root.wm_title("Furuta Pendulum Tester [ by j :) ]" ) #: overall ttk window title
        self.root.geometry('1500x1000') #: ttk frame size
        # self.build_serial_port() #: Call the serial port setup function
        self.build_controls() #: Call the control controls setup function
        self.build_calibration() #: Call the calibration controls setup function
        self.build_plot() #: Call the plot panel setup function
        self.after_id1 = self.root.after(100, self.read_serial) #: After 100ms, start reading the serial data
        self.after_id2 = self.root.after(500, self.plot_graph) #: After 500ms, start plotting on the graphs
        self.root.mainloop() #: 
           
    def build_serial_port(self):
        """
        Builds and initializes the serial port for communication with the pendulum

        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        self.t0 = time.time()
        self.serial_frame = ttk.Frame(self.root, padding = 10)
        self.serial_frame.grid(column = 0, row = 0, sticky = "ew")
        self.ser = serial.Serial(port = '/dev/tty.usbmodem144403', baudrate = 921600, bytesize = serial.EIGHTBITS, dsrdtr = True, timeout = 1)  # open serial port
        print("DSR = ", self.ser.getDSR())
        self.ser.setDTR(False)
        time.sleep(0.2)
        self.ser.setDTR(True)
        self.ser.reset_input_buffer()
        self.ser.flushInput()
        self.ser.flushOutput()
        print("com port used:", self.ser.name)         # check which port was really used

    def build_controls(self):
        """
        Initializes the frame in the tester for the controls interface

        :param kind: None
        :type kind: None
        :raise none
        :return: none
        :rtype: none

        """

        self.controls_power_frame = ttk.Frame(self.root) #: Create a new frame for power controls
        
        style = ttk.Style()
        style.theme_use('darkly')
        self.serial_process_timing = ttk.StringVar()
        self.serial_process_timing.set("hi")
        self.serial_read_speed_label = ttk.Label(self.controls_power_frame, textvariable=self.serial_process_timing)
        self.controls_power_frame.grid(column = 1, row = 0)
        
        controls_power_label = ttk.Label(self.controls_power_frame,text="UI Controls",font=("Helvetica", 12, "bold"))
        self.on_button = ttk.Button(self.controls_power_frame,  text = 'Turn on (press a)', bootstyle = 'success', command = self.on) #: "On" button for the pendulum
        self.reset_graph_button = ttk.Button(self.controls_power_frame,  text = 'Reset Graph (press d)', bootstyle = 'primary', command = self.reset_graph) #: "Reset graph" button for the testing interface
        self.off_button = ttk.Button(self.controls_power_frame, text = 'Turn off (press s)', bootstyle = 'danger', command = self.off) #: "Off" button for the pendulum
        self.button_quit = ttk.Button(self.controls_power_frame, text="Quit (press q)", command=self.end_tester)
        controls_power_label.pack(padx=10, pady=10) # Center the label within the frame

        self.on_button.pack(side = LEFT, padx=10, pady=10)
        self.off_button.pack(side = RIGHT, padx=10, pady=10)
        self.reset_graph_button.pack(padx=10, pady=10)
        self.button_quit.pack(padx=10, pady=10)
        self.serial_read_speed_label.pack()

        self.controls_control_frame = ttk.Frame(self.root)
        style = ttk.Style()
        style.theme_use('darkly')
        self.controls_control_frame.grid(column = 1, row = 1)
        controls_control_label = ttk.Label(self.controls_control_frame,text="Pendulum Controls",font=("Helvetica", 12, "bold"))
        self.motor_forward_button = ttk.Button(self.controls_control_frame, text="Motor Forward (press j)", command=self.motor_forward)
        self.motor_backward_button = ttk.Button(self.controls_control_frame, text="Motor Backward (press k)", command=self.motor_backward)
        self.tweaked_controls_button = ttk.Button(self.controls_control_frame, text="Tweaked Controls (press t)", command=self.tweaked_controls)
        controls_control_label.pack(padx=10, pady=10) # Center the label within the frame
        self.motor_forward_button.pack()
        self.motor_backward_button.pack()
        self.tweaked_controls_button.pack()

        
        self.root.bind('<KeyPress-a>', lambda e: self.on())
        self.root.bind('<KeyPress-s>', lambda e: self.off())
        self.root.bind('<KeyPress-q>', lambda e: self.end_tester())
        self.root.bind('<KeyPress-d>', lambda e: self.reset_graph())
        self.root.bind('<KeyPress-j>', lambda e: self.motor_forward())
        self.root.bind('<KeyPress-k>', lambda e: self.motor_backward())
        self.root.bind('<KeyPress-t>', lambda e: self.tweaked_controls())
        
    def build_calibration(self):
            """
            Initializes the frame in the interface for the calibration buttons (Motor and model calibration)

            :param kind: Optional "kind" of ingredients.
            :type kind: list[str] or None
            :raise none
            :return: none
            :rtype: none

            """
            self.calibration_frame = ttk.Frame(self.root, padding = 10)
            self.calibration_frame.grid(column = 1, row = 2, sticky = "ew")
            self.calibration_label = ttk.Label(self.calibration_frame,text="Calibration Controls",font=("Helvetica", 12, "bold"))
            self.model_calibrate_button = ttk.Button(self.calibration_frame, text="Calibrate Model (press c)", bootstyle = 'info', command=self.model_calibrate)
            self.motor_calibrate_button = ttk.Button(self.calibration_frame, text="Calibrate Motor (press m)", bootstyle = 'info', command=self.model_calibrate)
            self.root.bind('<KeyPress-c>', lambda e: self.model_calibrate())
            self.root.bind('<KeyPress-m>', lambda e: self.motor_calibrate())
            self.calibration_label.pack()
            self.model_calibrate_button.pack()
            self.motor_calibrate_button.pack()
        
    def build_plot(self):
        """
        Initializes the frame in the interface for the plots :)

        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        self.plot_frame = ttk.Frame(self.root, padding = 10)
        self.plot_frame.grid(column = 0, row = 0, rowspan=3,sticky = "ew")
        self.plot_label = ttk.Label(self.plot_frame,text="Plots",font=("Helvetica", 12, "bold"))
        self.fig = figure.Figure(figsize = [9,9])
        self.axs  = self.fig.subplots(5,1)
        print(self.axs)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)  # A tk.DrawingArea.
        self.canvas.draw()
        self.plot_label.pack(padx=10, pady=10) # Center the label within the frame
        self.plot_timing = ttk.StringVar()
        self.plot_timing.set("hi")
        self.plot_draw_speed_label = ttk.Label(self.plot_frame, textvariable=self.plot_timing).pack()
        self.canvas.get_tk_widget().pack()
        self.plot_graph()

    def tweaked_controls(self):
        self.ser.write('t'.encode('utf-8'))
        print("tweaked controls selected")

    def motor_forward(self):
        self.ser.write('j'.encode('utf-8'))
        print("motor forward")

    def motor_backward(self):
        self.ser.write('k'.encode('utf-8')) 
        print("motor backward")

    def motor_calibrate(self):
        self.ser.write('m'.encode('utf-8')) 
        print("motor calibrating")
        
    def end_tester(self):
        """
        Closes the testing window and ends the program

        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        self.ser.close()
        print('serial closed')
        self.root.after_cancel(self.after_id)
        print('after cancelled id:', self.after_id)
        self.root.after(100,self.root.destroy)
        print('root destroyed :D')
    
    def calculate_model(self):
        """
        Initializes model calibration for the pendulum, then calculates the model once all of the necessary data has been collected.

        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        print("model calculation started")
        self.calibration_flag = 0
        print("calibration_flag = 0")
        self.state_0 = np.array([self.motor_angle[0:198], self.pendulum_angle[0:198], self.motor_speed[0:198], self.pendulum_speed[0:198], self.motor_effort[0:198]])
        print("state 0 made", self.state_0)
        self.state_1 = (np.array([self.motor_angle[1:199], self.pendulum_angle[1:199],  self.motor_speed[1:199], self.pendulum_speed[1:199]]) - self.state_0[[0,1,2,3], :])/0.02
        print("state 1 made")
        state_t = self.state_0 @ self.state_0.T
        print("square state thing:", state_t)
        try: 
            self.state_space = (np.linalg.inv(self.state_0 @ self.state_0.T) @ self.state_0) @ self.state_1.T
            print("state space:", self.state_space.T)
        except:
            print("Singular Matrix, no dice")
        x_train = self.state_0
        y_train = self.state_1[:][0]
        xlag = [list(range(1, 2))] * x_train.shape[1]
        basis_function = Polynomial(degree=2)
        estimator = LeastSquares()
        model = FROLS(
            n_info_values=3,
            ylag=4,
            xlag=xlag,
            estimator=estimator,
            basis_function=basis_function,
        )
        model.fit(X=x_train, y=y_train)
        # print the identified model
        r = pd.DataFrame(
            results(
                model.final_model,
                model.theta,
                model.err,
                model.n_terms
            ),
            columns=["Regressors", "Parameters", "ERR"],
        )
        print(r)
        

    def on(self):
        print("on")

    def reset_graph(self):
        """
        Resets the graph and clears all data fields.
        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        self.i = 0
        if self.pendulum_speed:
            print("pendulum speed/// max:", max(self.pendulum_speed), "min:", min(self.pendulum_speed))
        self.pendulum_angle.clear()
        self.motor_angle.clear()
        self.motor_speed.clear()
        self.pendulum_speed.clear()
        self.motor_effort.clear()
        self.t.clear()

    def off(self):
        self.ser.write('s'.encode('utf-8')) 
        print("off")
    
    def model_calibrate(self):
        self.ser.write('c'.encode('utf-8')) 
        print("model calibration started")
        self.calibration_flag = 1
        self.reset_graph()
     
    def plot_graph(self):
        """
        Plots the graphs using the existing data

        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        start = time.perf_counter()
        for ax in self.axs:
            ax.clear()
            if ax == self.axs[0]:
                ax.plot(self.t, self.motor_angle)
                ax.set_ylabel('Motor Angle (rad)')
                ax.set_ylim(-math.pi, math.pi)
            if ax == self.axs[1]:
                ax.plot(self.t, self.pendulum_angle)
                ax.set_ylabel('Pendulum Angle (rad)')
                ax.set_ylim(-math.pi, math.pi)
            if ax == self.axs[2]:
                ax.plot(self.t, self.motor_speed)
                ax.set_ylabel('Motor Speed (rad/s)')
                ax.set_ylim(-10, 10)
            if ax == self.axs[3]:
                ax.plot(self.t, self.pendulum_speed)
                ax.set_ylabel('Pendulum Speed (rad/s)')
                ax.set_ylim(-20, 20)
            if ax == self.axs[4]:
                ax.plot(self.t, self.motor_effort)
                ax.set_ylabel('Motor Effort')
                ax.set_xlabel('Time, t (s)')
                ax.set_ylim(-120, 120)
            ax.grid()
            if self.t:
                ax.set_xlim(self.t[0],self.t[-1])
            
            # print(self.data_stripped)
            
        self.canvas.draw()
        if self.update_graph_flag == 1:
            self.after_id = self.root.after(500, self.plot_graph)
        end = time.perf_counter()
        self.plot_timing.set(f"Plot draw time: {(end-start)*1000:.3f} ms")
    
    def read_serial(self):
        """
        Reads the serial port, parses data, calculates physical values from data, and adds it to the data fields.

        :param kind: Optional "kind" of ingredients.
        :type kind: list[str] or None
        :raise none
        :return: none
        :rtype: none

        """
        # print("bytes waiting:", self.ser.in_waiting)
        start = time.perf_counter()
        if self.ser.in_waiting:
            self.data_raw = self.ser.read_all().decode('utf-8').split("\r\n")

            if self.data_raw != '':
                self.data_raw.pop(-1)

            for data in self.data_raw:
                nums = re.findall(r'-?\d+', data)
                # print("nums:", nums)
                if len(nums) == 6:
                    self.data_stripped.append(nums)
            
            if self.i == 0:
                self.pendulum_offset = int(self.data_stripped[0][0])
                self.motor_offset = int(self.data_stripped[0][1])
                self.t_offset = int(self.data_stripped[0][5])
            for data in self.data_stripped:
                if self.i == 200:
                    self.data_array = self.data_array[:,1:]
                    self.t.pop(0)
                    self.pendulum_angle.pop(0)
                    self.motor_angle.pop(0)
                    self.pendulum_speed.pop(0)
                    self.motor_speed.pop(0)
                    self.motor_effort.pop(0)
                
                self.t.append((int(data[5])-self.t_offset)/1000)
                self.pendulum_angle.append((((int(data[0])-self.pendulum_offset)+self.pendulum_encoder_ticks/2)%self.pendulum_encoder_ticks-self.pendulum_encoder_ticks/2)*2*math.pi/self.pendulum_encoder_ticks)
                self.motor_angle.append((((int(data[1])-self.motor_offset+self.motor_encoder_ticks/2)%self.motor_encoder_ticks)-self.motor_encoder_ticks/2)*2*math.pi/self.motor_encoder_ticks)
                self.pendulum_speed.append((int(data[2]))*2*math.pi/(self.pendulum_encoder_ticks))
                self.motor_speed.append((int(data[3]))*2*math.pi/(self.motor_encoder_ticks))
                self.motor_effort.append(int(data[4]))
                appended_data = np.append(self.pendulum_angle, self.motor_angle, self.pendulum_speed, self.motor_speed, self.motor_effort, axis=0)
                self.data_array= np.concatenate(self.data_array, appended_data, axis = 1)

                if self.i<200:
                    self.i+=1
                if (self.i == 200 and self.calibration_flag == 1):
                    self.calculate_model()
        end = time.perf_counter()
        self.serial_process_timing.set(f"Serial reading time: {(end-start)*1000:.3f} ms")
        self.data_stripped.clear()
        self.after_id = self.root.after(50, self.read_serial)

tester = FurutaTester()
