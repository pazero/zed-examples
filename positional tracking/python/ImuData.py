class ImuData:
    def __init__(self, imu_Timestamp, mag_Timestamp, baro_Timestamp, accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ, orX, orY, orZ, press, rel_alt, moving, temp_left, temp_right, temp_imu, temp_barom):
        self.imu_Timestamp = str(imu_Timestamp)
        self.mag_Timestamp = str(mag_Timestamp)
        self.baro_Timestamp = str(baro_Timestamp)
        self.accX = str(accX)
        self.accY = str(accY)
        self.accZ = str(accZ)
        self.gyroX = str(gyroX)
        self.gyroY = str(gyroY)
        self.gyroZ = str(gyroZ)
        self.magX = str(magX)
        self.magY = str(magY)
        self.magZ = str(magZ)
        self.orX = str(orX)
        self.orY = str(orY)
        self.orZ = str(orZ)
        self.press = str(press)
        self.rel_alt = str(rel_alt)
        self.moving = str(moving)
        self.temp_left = str(temp_left)
        self.temp_right = str(temp_right)
        self.temp_imu = str(temp_imu)
        self.temp_barom = str(temp_barom)
