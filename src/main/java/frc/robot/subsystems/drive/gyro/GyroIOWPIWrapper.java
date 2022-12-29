package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class GyroIOWPIWrapper implements GyroIO {
    private Gyro m_gyro;
    private Rotation2d m_offset;

    public GyroIOWPIWrapper(Gyro gyro) {
        m_gyro = gyro;
        m_offset = Rotation2d.fromDegrees(0);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.rawAngleRads = getRawGyroAngle().getRadians();
        inputs.angleRads = getRotation2d().getRadians();
        inputs.rateRadsPerSec = getRate();
    }

    @Override
    public Gyro getWPIGyro() {
        return m_gyro;
    }

    @Override
    public Rotation2d getRawGyroAngle() {
        return m_gyro.getRotation2d();
    }

    @Override
    public Rotation2d getOffset() {
        return m_offset;
    }

    @Override
    public double getRate() {
        return m_gyro.getRate();
    }

    @Override
    public void reset() {
        reset(Rotation2d.fromDegrees(0));
    }

    @Override
    public void reset(Rotation2d offset) {
        m_gyro.reset();
        m_offset = offset;
    }

    @Override
    public void addAngle(Rotation2d angle) {
        m_offset = m_offset.plus(angle);
    }
}
