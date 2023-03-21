class ImuData:
    def __init__(self, imu_Timestamp, mag_Timestamp, baro_Timestamp, accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ, orX, orY, orZ, press, rel_alt, moving, temp_left, temp_right, temp_imu, temp_barom):
        self.imu_Timestamp = imu_Timestamp
        self.mag_Timestamp = mag_Timestamp
        self.baro_Timestamp = baro_Timestamp
        self.accX = accX
        self.accY = accY
        self.accZ = accZ
        self.gyroX = gyroX
        self.gyroY = gyroY
        self.gyroZ = gyroZ
        self.magX = magX
        self.magY = magY
        self.magZ = magZ
        self.orX = orX
        self.orY = orY
        self.orZ = orZ
        self.press = press
        self.rel_alt = rel_alt
        self.moving = moving
        self.temp_left = temp_left
        self.temp_right = temp_right
        self.temp_imu = temp_imu
        self.temp_barom = temp_barom
