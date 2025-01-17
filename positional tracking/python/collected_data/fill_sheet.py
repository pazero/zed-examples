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
import datetime
import xlsxwriter
from varname import nameof
import ImuData
import math
import keyboard

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
        if isinstance(sensor, sl.IMUData):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_
        elif isinstance(sensor, sl.MagnetometerData):
            new_ = (sensor.timestamp.get_microseconds() > self.t_mag.get_microseconds())
            if new_:
                self.t_mag = sensor.timestamp
            return new_
        elif isinstance(sensor, sl.BarometerData):
            new_ = (sensor.timestamp.get_microseconds() > self.t_baro.get_microseconds())
            if new_:
                self.t_baro = sensor.timestamp
            return new_


def printSensorParameters(sensor_parameters):
    if sensor_parameters.is_available:
        print("*****************************")
        print("Sensor type: " + str(sensor_parameters.sensor_type))
        print("Max rate: "  + str(sensor_parameters.sampling_rate) + " "  + str(sl.SENSORS_UNIT.HERTZ))
        print("Range: "  + str(sensor_parameters.sensor_range) + " "  + str(sensor_parameters.sensor_unit))
        print("Resolution: " + str(sensor_parameters.resolution) + " "  + str(sensor_parameters.sensor_unit))
        if not math.isnan(sensor_parameters.noise_density):
            print("Noise Density: "  + str(sensor_parameters.noise_density) + " " + str(sensor_parameters.sensor_unit) + "/√Hz")
        if not math.isnan(sensor_parameters.random_walk):
            print("Random Walk: "  + str(sensor_parameters.random_walk) + " " + str(sensor_parameters.sensor_unit) + "/s/√Hz")

# def dot2comma(str):

def write_xlsx(sheet_list):
    now = str(datetime.datetime.now().day) + '/'
    now += str(datetime.datetime.now().month) + '/'
    now += str(datetime.datetime.now().year) + ' '
    now += str(datetime.datetime.now().hour) + ':'
    mins = datetime.datetime.now().minute
    now += str(mins) if mins > 9 else '0' + str(mins)
    print(now)
    if len(sheet_list) == 0:
        return
    imu = sheet_list[0]

    workbook = xlsxwriter.Workbook('data.xlsx')

    # The workbook object is then used to add new worksheet via the add_worksheet() method.
    worksheet = workbook.add_worksheet()

    worksheet.write('A1', 'v2')
    worksheet.write('B1', now)
    worksheet.write('A2', nameof(imu.imu_Timestamp) + '[sec]')
    worksheet.write('B2', nameof(imu.mag_Timestamp) + '[sec]')
    worksheet.write('C2', nameof(imu.baro_Timestamp) + '[sec]')
    worksheet.write('D2', nameof(imu.accX) + '[m/s^2]')
    worksheet.write('E2', nameof(imu.accY) + '[m/s^2]')
    worksheet.write('F2', nameof(imu.accZ) + '[m/s^2]')
    worksheet.write('G2', nameof(imu.gyroX) + '[deg/s]')
    worksheet.write('H2', nameof(imu.gyroY) + '[deg/S]')
    worksheet.write('I2', nameof(imu.gyroZ) + '[deg/s]')
    worksheet.write('J2', nameof(imu.magX) + '[uT]')
    worksheet.write('K2', nameof(imu.magY) + '[uT]')
    worksheet.write('L2', nameof(imu.magZ) + '[uT]')
    worksheet.write('M2', nameof(imu.orX) + '[deg]')
    worksheet.write('N2', nameof(imu.orY) + '[deg]')
    worksheet.write('O2', nameof(imu.orZ) + '[deg]')
    worksheet.write('P2', nameof(imu.press) + '[hPa]')
    worksheet.write('Q2', nameof(imu.rel_alt) + '[m]')
    worksheet.write('R2', nameof(imu.moving))
    worksheet.write('S2', nameof(imu.temp_left) + '[C]')
    worksheet.write('T2', nameof(imu.temp_right) + '[C]')
    worksheet.write('U2', nameof(imu.temp_imu) + '[C]')
    worksheet.write('V2', nameof(imu.temp_barom) + '[C]')

    riga = 3
    for i in sheet_list:
        worksheet.write('A' + str(riga), i.imu_Timestamp)
        worksheet.write('B' + str(riga), i.mag_Timestamp)
        worksheet.write('C' + str(riga), i.baro_Timestamp)
        worksheet.write('D' + str(riga), i.accX)
        worksheet.write('E' + str(riga), i.accY)
        worksheet.write('F' + str(riga), i.accZ)
        worksheet.write('G' + str(riga), i.gyroX)
        worksheet.write('H' + str(riga), i.gyroY)
        worksheet.write('I' + str(riga), i.gyroZ)
        worksheet.write('J' + str(riga), i.magX)
        worksheet.write('K' + str(riga), i.magY)
        worksheet.write('L' + str(riga), i.magZ)
        worksheet.write('M' + str(riga), i.orX)
        worksheet.write('N' + str(riga), i.orY)
        worksheet.write('O' + str(riga), i.orZ)
        worksheet.write('P' + str(riga), i.press)
        worksheet.write('Q' + str(riga), i.rel_alt)
        worksheet.write('R' + str(riga), i.moving)
        worksheet.write('S' + str(riga), i.temp_left)
        worksheet.write('T' + str(riga), i.temp_right)
        worksheet.write('U' + str(riga), i.temp_imu)
        worksheet.write('V' + str(riga), i.temp_barom)

        riga += 1

    workbook.close()


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
    """
    printSensorParameters(info.sensors_configuration.accelerometer_parameters)  # accelerometer configuration
    printSensorParameters(info.sensors_configuration.gyroscope_parameters)  # gyroscope configuration
    printSensorParameters(info.sensors_configuration.magnetometer_parameters)  # magnetometer configuration
    printSensorParameters(info.sensors_configuration.barometer_parameters)  # barometer configuration
    """
    data_list = []
    ts_handler = TimestampHandler()
    sensors_data = sl.SensorsData()
    i = 0
    while True:
        if keyboard.is_pressed('enter'):
            break
        if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            # Check if the data has been updated since the last time. IMU is the sensor with the highest rate
            imu_data = sensors_data.get_imu_data()
            if ts_handler.is_new(imu_data):
                # da prendere: quaternioni, translation e imu_orientation
                #print(imu_data.timestamp.data_ns)
                imu_or = imu_data.get_pose().get_orientation().get()
                imu_vel = imu_data.get_angular_velocity()
                imu_acc = imu_data.get_linear_acceleration()
                mag_data = sensors_data.get_magnetometer_data().get_magnetic_field_calibrated()
                # non restituisce timestamp i dati nella forma giusta
                imuT = 0.000000001 * imu_data.timestamp.data_ns
                # print(imuT)
                magT = 0.000000001 * sensors_data.get_magnetometer_data().timestamp.data_ns
                barT = 0.000000001 * sensors_data.get_barometer_data().timestamp.data_ns
                accX = imu_acc[0]
                accY = imu_acc[1]
                accZ = imu_acc[2]
                gyrX = imu_vel[0]
                gyrY = imu_vel[1]
                gyrZ = imu_vel[2]
                magX = mag_data[0]
                magY = mag_data[1]
                magZ = mag_data[2]
                orX = imu_or[0]
                orY = imu_or[1]
                orZ = imu_or[2]
                #prova
                press = sensors_data.get_barometer_data().pressure
                # ? sembra che altitudine relativa sia sempre 0.0, nonostante sposti la telecamera in alto
                r_alt = sensors_data.get_barometer_data().relative_altitude
                # non restituisce il formato giusto. Inoltre quando sta ferma sembra che dica che si muove
                # mov è 0 se telecamera è ferma, 1 se si muove e -1 se sta cadendo
                mov = 0 if sensors_data.camera_moving_state == sl.CAMERA_MOTION_STATE.STATIC else 1 if sensors_data.camera_moving_state == sl.CAMERA_MOTION_STATE.MOVING else -1
                #if sensors_data.camera_moving_state == sl.CAMERA_MOTION_STATE.FALLING:
                #    mov = -1
                tLeft = sensors_data.get_temperature_data().get(sl.SENSOR_LOCATION.ONBOARD_LEFT)
                tRight = sensors_data.get_temperature_data().get(sl.SENSOR_LOCATION.ONBOARD_RIGHT)
                tImu = sensors_data.get_temperature_data().get(sl.SENSOR_LOCATION.IMU)
                tBar = sensors_data.get_temperature_data().get(sl.SENSOR_LOCATION.BAROMETER)
                data_list.append(ImuData.ImuData(imuT, magT, barT, accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ, orX, orY, orZ, press, r_alt, mov, tLeft, tRight, tImu, tBar))
                i += 1
    print(len(data_list))
    write_xlsx(data_list)
    zed.close()
    return 0


if __name__ == "__main__":
    main()

