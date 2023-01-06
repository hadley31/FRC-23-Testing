package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class PigeonIO extends GyroIOWPIWrapper {
    public PigeonIO(int port) {
        super(new WPI_Pigeon2(port));
    }

    @Override
    public WPI_Pigeon2 getWPIGyro() {
        return (WPI_Pigeon2) super.getWPIGyro();
    }
}
