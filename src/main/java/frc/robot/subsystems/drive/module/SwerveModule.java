package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.advantagekit.LoggedIOContainer;

public class SwerveModule implements LoggedIOContainer<SwerveModuleIO> {
  public static final String[] kModuleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };
  public static final double kMaxTurnRate = Units.degreesToRadians(2000);
  public static final double kMaxTurnAccel = Units.degreesToRadians(800);

  private final String m_name;

  private final SwerveModuleIO m_io;
  private final SwerveModuleInputsAutoLogged m_inputs;

  private final SimpleMotorFeedforward m_turnFF;
  private final ProfiledPIDController m_turnController;

  private final SimpleMotorFeedforward m_driveFF;
  private final PIDController m_driveController;

  private SwerveModuleState m_desiredState;

  public SwerveModule(String moduleName, SwerveModuleIO io) {
    m_name = moduleName;

    m_io = io;
    m_inputs = new SwerveModuleInputsAutoLogged();

    m_turnFF = new SimpleMotorFeedforward(0.1, 0.1, 0.1);
    m_turnController = new ProfiledPIDController(0.8, 0.005, 0.05, new Constraints(kMaxTurnRate, kMaxTurnAccel));
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnController.setTolerance(kMaxTurnRate, kMaxTurnAccel);

    m_driveFF = new SimpleMotorFeedforward(0.1, 0.1, 0.1);
    m_driveController = new PIDController(0.1, 0.1, 0.1);

    m_desiredState = getState();
  }

  public void update() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs(m_name, m_inputs);

    // Calculate turn voltages
    double desiredAngleRad = MathUtil.angleModulus(m_desiredState.angle.getRadians());

    double turnPIDVoltage = m_turnController.calculate(m_inputs.turnPositionRad, desiredAngleRad);
    double turnFFVoltage = m_turnFF.calculate(m_turnController.getSetpoint().velocity);

    double turnOutputVoltage = turnFFVoltage + turnPIDVoltage;

    // Calculate drive voltages
    double desiredSpeed = m_desiredState.speedMetersPerSecond;
    double drivePIDVoltage = m_driveController.calculate(m_inputs.driveVelocityMetersPerSec, desiredSpeed);
    double driveFFVoltage = m_driveFF.calculate(desiredSpeed);

    double driveOutputVoltage = driveFFVoltage + drivePIDVoltage;

    // Apply voltages
    m_io.setTurnVoltage(turnOutputVoltage);
    m_io.setDriveVoltage(driveOutputVoltage);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    m_desiredState = desiredState;
  }

  public Rotation2d getRotation() {
    return Rotation2d.fromRadians(m_inputs.turnPositionRad);
  }

  public Rotation2d getAbsoluteRotation() {
    return Rotation2d.fromRadians(m_inputs.turnAbsolutePositionRad);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_inputs.driveVelocityMetersPerSec, getRotation());
  }

  public SwerveModuleState getAbsoluteState() {
    return new SwerveModuleState(m_inputs.driveVelocityMetersPerSec, getAbsoluteRotation());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_inputs.drivePositionMeters, getRotation());
  }

  public SwerveModulePosition getAbsolutePosition() {
    return new SwerveModulePosition(m_inputs.drivePositionMeters, getAbsoluteRotation());
  }

  public SwerveModuleIO getIO() {
    return m_io;
  }
}
