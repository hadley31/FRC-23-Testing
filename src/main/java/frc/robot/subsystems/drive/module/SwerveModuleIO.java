package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.drive.module.SwerveModuleIO.SwerveModuleInputs;

public interface SwerveModuleIO extends LoggedIO<SwerveModuleInputs> {
  @AutoLog
  public static class SwerveModuleInputs {
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelcius = 0.0;

    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelcius = 0.0;
  }

  public void zeroTurnEncoder();

  public void zeroDriveEncoder();

  public default void zeroTurnAbsoluteEncoder() {
    DriverStation.reportWarning("This module does not support zeroing the absolute encoder", false);
  }

  public void syncTurnEncoderWithAbsolute();

  public void setTargetTurnPosition(Rotation2d positionRadians);

  public void setTargetDriveVelocity(double velocityMetersPerSecond);

  public void setTurnBrakeMode(boolean brake);

  public void setDriveBrakeMode(boolean brake);
}
