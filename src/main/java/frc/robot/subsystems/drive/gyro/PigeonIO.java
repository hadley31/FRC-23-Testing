package frc.robot.subsystems.drive.gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

public class PigeonIO extends GyroWrapper {
    public PigeonIO(int port) {
        super(new WPI_Pigeon2(port));
    }
}
