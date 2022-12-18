package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.LoggableInputs;
import frc.robot.subsystems.drive.gyro.GyroIO.GyroInputs;

public interface GyroIO extends LoggableInputs<GyroInputs> {
    @AutoLog
    public static class GyroInputs {
        public double rawAngleRads;
        public double angleRads;
        public double rateRadsPerSec;
    }

    public Rotation2d getRawGyroAngle();

    public default Rotation2d getRotation2d() {
        return getRawGyroAngle().plus(getOffset());
    }

    public Rotation2d getOffset();

    public double getRate();

    public void reset();

    public void reset(Rotation2d offset);

    public void addAngle(Rotation2d angle);

    public Gyro getWPIGyro();
}
