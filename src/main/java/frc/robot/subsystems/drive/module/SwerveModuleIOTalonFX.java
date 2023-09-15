package frc.robot.subsystems.drive.module;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.lib.utils.TalonFXUtils;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  private static final double kDriveConversionFactor = 1.0; // TODO
  private static final double kTurnConversionFactor = 1.0; // TODO
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turnMotor;

  private final Rotation2d m_absoluteOffset;

  private final AnalogEncoder m_turnAbsoluteEncoder;

  public SwerveModuleIOTalonFX(int turnMotorPort, int driveMotorPort, int turnAbsoluteEncoderPort,
      Rotation2d offset) {
    // Define Motors
    this.m_turnMotor = new WPI_TalonFX(turnMotorPort);
    this.m_driveMotor = new WPI_TalonFX(driveMotorPort);

    this.m_absoluteOffset = offset;

    // Set Idle Modes
    this.m_turnMotor.setNeutralMode(NeutralMode.Brake);
    this.m_driveMotor.setNeutralMode(NeutralMode.Coast);

    // m_turnMotor.setSmartCurrentLimit(30);
    // m_driveMotor.setSmartCurrentLimit(30);

    m_turnMotor.setInverted(false);
    m_driveMotor.setInverted(false);

    m_turnAbsoluteEncoder = new AnalogEncoder(turnAbsoluteEncoderPort);
    m_turnAbsoluteEncoder.setDistancePerRotation(DriveConstants.kTurnPositionConversionFactor);

    TalonFXUtils.setDefaultPIDF(m_driveMotor, 1.0, 0.0, 0.0, 1.0);
    TalonFXUtils.setDefaultPIDF(m_turnMotor, 1.0, 0.0, 0.0, 1.0);
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    inputs.driveAppliedVolts = m_driveMotor.getMotorOutputVoltage();
    inputs.drivePositionMeters = getDrivePositionMeters();
    inputs.driveVelocityMetersPerSec = getVelocityMetersPerSecond();
    inputs.driveCurrentAmps = m_driveMotor.getStatorCurrent();
    inputs.driveTempCelcius = m_driveMotor.getTemperature();

    inputs.turnAppliedVolts = m_turnMotor.getMotorOutputVoltage();
    inputs.turnAbsolutePositionRad = getAbsoluteRotation().getRadians();
    inputs.turnPositionRad = getRotation().getRadians();
    inputs.turnVelocityRadPerSec = getTurnVelocityRadPerSecond();
    inputs.turnCurrentAmps = m_turnMotor.getStatorCurrent();
    inputs.turnTempCelcius = m_turnMotor.getTemperature();
  }

  @Override
  public void zeroTurnEncoder() {
    m_turnMotor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void zeroDriveEncoder() {
    m_driveMotor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void syncTurnEncoderWithAbsolute() {
    double offset = TalonFXUtils.rotation2dToTicks(getAbsoluteRotation(), kTurnConversionFactor);
    m_driveMotor.setSelectedSensorPosition(offset);
  }

  @Override
  public void setTargetTurnPosition(Rotation2d rotation2d) {
    double ticks = TalonFXUtils.rotation2dToTicks(rotation2d, kTurnConversionFactor);
    m_turnMotor.set(TalonFXControlMode.Position, ticks);
  }

  @Override
  public void setTargetDriveVelocity(double velocityMetersPerSecond) {
    m_driveMotor.set(TalonFXControlMode.Velocity, velocityMetersPerSecond);
  }

  @Override
  public void setTurnBrakeMode(boolean brake) {
    m_turnMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  @Override
  public void setDriveBrakeMode(boolean brake) {
    m_driveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
  }

  private double getVelocityMetersPerSecond() {
    double ticksPer100ms = m_driveMotor.getSelectedSensorVelocity();
    return TalonFXUtils.ticksPer100msToMetersPerSecond(ticksPer100ms, kDriveConversionFactor);
  }

  private double getTurnVelocityRadPerSecond() {
    double ticksPer100ms = m_turnMotor.getSelectedSensorVelocity();
    return TalonFXUtils.ticksPer100msToRadsPerSecond(ticksPer100ms, kTurnConversionFactor);
  }

  private Rotation2d getRotation() {
    double ticks = m_turnMotor.getSelectedSensorPosition();
    double radians = TalonFXUtils.ticksToRadians(ticks, kTurnConversionFactor);
    return Rotation2d.fromRadians(MathUtil.angleModulus(radians));
  }

  private Rotation2d getAbsoluteRotation() {
    double radians = (1.0 - m_turnAbsoluteEncoder.getAbsolutePosition()) * 2 * Math.PI;
    return Rotation2d.fromRadians(MathUtil.angleModulus(radians)).plus(m_absoluteOffset);
  }

  private double getDrivePositionMeters() {
    return TalonFXUtils.ticksToMeters(m_driveMotor.getSelectedSensorPosition(), kDriveConversionFactor);
  }
}
