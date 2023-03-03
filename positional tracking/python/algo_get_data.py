########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import xlsxwriter
import array as arr
import cv2
import numpy as np
import math
import time
import locale
#locale.setlocale(locale.LC_ALL, 'nl_NL')
##
# Basic class to handle the timestamp of the different sensors to know if it is a new sensors_data or an old one
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()
        self.t_baro = sl.Timestamp()
        self.t_mag = sl.Timestamp()

    ##
    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if (isinstance(sensor, sl.IMUData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.MagnetometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_mag.get_microseconds())
            if new_:
                self.t_mag = sensor.timestamp
            return new_
        elif (isinstance(sensor, sl.BarometerData)):
            new_ = (sensor.timestamp.get_microseconds() > self.t_baro.get_microseconds())
            if new_:
                self.t_baro = sensor.timestamp
            return new_


##

def main():
    # Create a Camera object
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NONE

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(err))
        zed.close()
        exit(1)

    # Get camera information sensors_data
    info = zed.get_camera_information()

    cam_model = info.camera_model
    if cam_model == sl.MODEL.ZED:
        print("This tutorial only supports ZED-M and ZED2 camera models, ZED does not have additional sensors")
        exit(1)

    # Used to store the sensors timestamp to know if the sensors_data is a new one or not
    ts_handler = TimestampHandler()

    i = 0
    sensors_data = sl.SensorsData()
    x_translation_imu_data = ["x_translation"]
    y_translation_imu_data = ["y_translation"]
    z_translation_imu_data = ["z_translation"]
    x_orientation_imu_data = ["x_orientation"]
    y_orientation_imu_data = ["y_orientation"]
    z_orientation_imu_data = ["z_orientation"]
    workbook = xlsxwriter.Workbook('algo_data.xlsx')
    worksheet = workbook.add_worksheet()
    while i < 10000:
        if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            # Check if the data has been updated since the last time. IMU is the sensor with the highest rate
            if ts_handler.is_new(sensors_data.get_imu_data()):
                # GET quaternioni, translation e imu_orientation
                # print("Sample " + str(i))
                imu_pose = sensors_data.get_imu_data().get_pose()
                imu_orientation = imu_pose.get_orientation().get()
                imu_translation = imu_pose.get_translation().get()
                print("\t Translation: ", imu_translation, "\t Orientation: ", imu_orientation())
                x_orientation_imu_data.append(imu_orientation[0])
                y_orientation_imu_data.append(imu_orientation[1])
                z_orientation_imu_data.append(imu_orientation[2])
                x_translation_imu_data.append(imu_translation[0])
                y_translation_imu_data.append(imu_translation[1])
                z_translation_imu_data.append(imu_translation[2])
                
                # Filtered orientation quaternion
                quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
                print(" \t Orientation: [ Ox: {0}, Oy: {1}, Oz {2}, Ow: {3} ]".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

                i = i + 1
                #time.sleep(.3)
                # input("Press Enter to continue...")
        print("x orie:", x_orientation_imu_data)
        print("y orie:", y_orientation_imu_data)
        print("z orie:", z_orientation_imu_data)
        print("x tran:", x_translation_imu_data)
        print("y tran:", y_translation_imu_data)
        print("z tran:", z_translation_imu_data)
    zed.close()
    return 0


if __name__ == "__main__":
    main()

