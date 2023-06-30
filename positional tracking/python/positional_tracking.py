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

"""
    This sample shows how to track the position of the ZED camera 
    and displays it in a OpenGL window.
"""

import sys
import ogl_viewer.tracking_viewer as gl
import pyzed.sl as sl

def print_pose_value(pose):
    print("\npose: ", pose)
    timestamp = pose.timestamp.get_nanoseconds()
    print("timestamp: ", timestamp)
    print("confidence: ", pose.pose_confidence)
    print("valido: ", pose.valid)

    rotation = pose.get_rotation_vector()
    translation = pose.get_translation().get()
    print("rotation: ", rotation)
    print("translation: ", translation)
    #print("covarianza: ", pose.pose_covariance)


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

def reboot_btn():
    print("import FUNZIONA")
    # zed.reboot(0, true)


if __name__ == "__main__":
    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP)
                                 
    # If applicable, use the SVO given as parameter
    # Otherwise use ZED live stream
    if len(sys.argv) == 2:
        filepath = sys.argv[1]
        print("Using SVO file: {0}".format(filepath))
        init_params.set_from_svo_file(filepath)

    zed = sl.Camera()

    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()
    #tracking_params.confidence_threshold = 100;
    tracking_params = sl.PositionalTrackingParameters()
    #tracking_params.enable_area_memory = False
    mem_area = False if tracking_params.enable_area_memory is False else True

    #tracking_params.area_file_path = "C:\\Users\\Paolo\\Documents\\zed-examples\\positional tracking\\memorized_area"
    #tracking_params.area_file_path = "./my_area"
    zed.enable_positional_tracking(tracking_params)
    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = 48
    runtime.texture_confidence_threshold = 39

    sensors_data = sl.SensorsData()
    camera_pose = sl.Pose()
    ts_handler = TimestampHandler()

    camera_info = zed.get_camera_information()
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    viewer.init(camera_info.camera_model)

    py_translation = sl.Translation()
    pose_data = sl.Transform()

    text_translation = ""
    text_rotation = ""
    text_accuracy = ""
    text_confidence = ""
    while viewer.is_available():
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            if zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
                if ts_handler.is_new(sensors_data.get_imu_data()):
                    linear_acceleration = sensors_data.get_imu_data().get_linear_acceleration()
                    #print(" \t Acceleration: [ {0:.3f} {1:.3f} {2:.3f} ] [m/sec^2]".format(linear_acceleration[0], linear_acceleration[1],linear_acceleration[2]))
                    angular_velocity = sensors_data.get_imu_data().get_angular_velocity()
                    text_velocity = str(("{:.3f}".format(angular_velocity[0]), "{:.3f}".format(angular_velocity[1]), "{:.3f}".format(angular_velocity[2])))
                    text_acceleration = str(("{:.3f}".format(linear_acceleration[0]), "{:.3f}".format(linear_acceleration[1]), "{:.3f}".format(linear_acceleration[2])))
                    #print(" \t Angular Velocities: [ {0:.3f} {1:.3f} {2:.3f} ] [deg/sec]".format(angular_velocity[0],angular_velocity[1],angular_velocity[2]))
                    #print("pose: {0}".format(sensors_data.get_imu_data().get_pose()))
            tracking_state = zed.get_position(camera_pose)
            #print_pose_value(camera_pose)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                rotation = camera_pose.get_rotation_vector()
                translation = camera_pose.get_translation(py_translation)
                text_rotation = str(("{:.3f}".format(rotation[0]), "{:.3f}".format(rotation[1]), "{:.3f}".format(rotation[2])))
                text_translation = str(("{:.3f}".format(translation.get()[0]), "{:.3f}".format(translation.get()[1]), "{:.3f}".format(translation.get()[2])))
                #text_accuracy = str(())
                pose_data = camera_pose.pose_data(sl.Transform())
            viewer.updateData(pose_data, text_translation, text_rotation, tracking_state, camera_pose.pose_confidence, mem_area, text_velocity, text_acceleration)

    viewer.exit()
    #zed.disable_positional_tracking(area_file_path="C:\\Users\\Paolo\\Documents\\zed-examples\\positional tracking\\memorized_area")
    #zed.save_area_map(area_file_path=".\\my_area")
    zed.close()