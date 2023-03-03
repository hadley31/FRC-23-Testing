package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.gyro.GyroIO;

public class ChargeStationBalance extends CommandBase {
  private static final double kMaxIncline = Units.degreesToRadians(10);
  private static final double kLevelThreshold = Units.degreesToRadians(2);
  private static final double kMaxBalanceSpeed = 1.0;
  private static final double kMountSpeed = 4.0;

  private final Drive m_drive;
  private final GyroIO m_gyro;
  private final PIDController m_controller;

  public ChargeStationBalance(Drive drive) {
    m_drive = drive;
    m_gyro = drive.getGyro();

    m_controller = new PIDController(0.3, 0.0, 0.05);

    m_controller.setTolerance(kLevelThreshold, Units.degreesToRadians(5));
    m_controller.setIntegratorRange(-0.2, 0.2);
  }

  @Override
  public void initialize() {
    m_drive.setDriveBrakeMode(true);
    m_drive.setTurnBrakeMode(true);

    m_controller.reset();
  }

  @Override
  public void execute() {
    Rotation2d angle = getLevelness();

    double xSpeed = m_controller.calculate(angle.getRadians(), 0);

    xSpeed = MathUtil.clamp(xSpeed, -kMaxBalanceSpeed, kMaxBalanceSpeed);

    var outputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 0, 0, m_drive.getPose().getRotation());

    m_drive.drive(outputSpeeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished!" + interrupted);
    m_drive.brake();
  }

  private Rotation2d getLevelness() {
    var upVector = m_gyro.getUpVector();

    // This gets the angle of the robot in the XZ plane
    // Calculates the angle between global up vector, and the up vector of the robot projected onto the XZ plane
    return new Rotation2d(upVector.get(2, 0), upVector.get(0, 0));
  }

  /**
   * Mount onto the charging station, then try to level out
   * @param drive
   * @return
   */
  public static CommandBase charge(Drive drive) {
    return Commands.sequence(
        mountChargingStation(drive),
        new ChargeStationBalance(drive));
  }

  /**
   * Drive forward until the drivebase is no longer level, or after 1.5 seconds
   * @param drive
   * @return
   */
  private static CommandBase mountChargingStation(Drive drive) {
    return drive
        .driveCommand(() -> {
          // Calculate where the robot is in relation to the charge station
          double robotOffsetX = FieldConstants.kChargeStationCenter.getX() - drive.getPose().getX();

          // Set the speed in that direction
          double speed = Math.copySign(kMountSpeed, robotOffsetX);
          return ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, 0, drive.getPose().getRotation());
        })
        .until(() -> !drive.getGyro().isLevel(kMaxIncline))
        .withTimeout(1.5);
  }
}
