package frc.robot.subsystems.drive.gyro;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.lib.advantagekit.LoggedIO;
import frc.lib.utils.GeometryUtil;
import frc.robot.subsystems.drive.gyro.GyroIO.GyroInputs;

public interface GyroIO extends LoggedIO<GyroInputs> {
  public static final Rotation2d kZero = Rotation2d.fromDegrees(0);

  @AutoLog
  public static class GyroInputs {
    public double rawAngleRads;
    public double angleRads;
    public double rateRadsPerSec;
    public double pitchRads;
    public double rollRads;
  }

  @Override
  public default void updateInputs(GyroInputs inputs) {
    inputs.rawAngleRads = getRawGyroAngle().getRadians();
    inputs.angleRads = getRotation2d().getRadians();
    inputs.rateRadsPerSec = getRate();
    inputs.pitchRads = getPitch().getRadians();
    inputs.rollRads = getRoll().getRadians();
  }

  public Rotation2d getRawGyroAngle();

  public default Rotation2d getRotation2d() {
    return getRawGyroAngle().plus(getOffset());
  }

  public Rotation2d getOffset();

  public double getRate();

  public void reset();

  public void reset(Rotation2d offset);

  public void addAngleOffset(Rotation2d angle);

  public void setAngleOffset(Rotation2d angle);

  public Gyro getWPIGyro();

  public default Rotation2d getPitch() {
    return getRawGyroPitch().plus(getPitchOffset());
  }

  public default Rotation2d getRoll() {
    return getRawGyroRoll().plus(getRollOffset());
  }

  public default Rotation2d getRawGyroPitch() {
    return kZero;
  }

  public default Rotation2d getRawGyroRoll() {
    return kZero;
  }

  public void setPitchOffset(Rotation2d pitchOffset);

  public void setRollOffset(Rotation2d rollOffset);

  public Rotation2d getPitchOffset();

  public Rotation2d getRollOffset();

  public default Rotation3d getOrientation() {
    return new Rotation3d(
        getRoll().getRadians(),
        getPitch().getRadians(),
        getRotation2d().getRadians());
  }

  public default Vector<N3> getUpVector() {
    return GeometryUtil.rotateVector(GeometryUtil.kUp, getOrientation().getQuaternion());
  }

  public default boolean isLevel(double thresholdRadians) {
    return Math.abs(getPitch().getRadians()) < thresholdRadians;
  }
}
