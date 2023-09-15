package frc.robot.subsystems.drive.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.advantagekit.LoggedIOContainer;

public class SwerveModule implements LoggedIOContainer<SwerveModuleIO> {
  public static final String[] kModuleNames = { "Front Left", "Front Right", "Back Left", "Back Right" };
  public static final double kMaxTurnRate = Units.degreesToRadians(2000);
  public static final double kMaxTurnAccel = Units.degreesToRadians(800);

  private final String m_name;

  private final SwerveModuleIO m_io;
  private final SwerveModuleInputsAutoLogged m_inputs;

  private SwerveModuleState m_desiredState;

  public SwerveModule(String moduleName, SwerveModuleIO io) {
    m_name = moduleName;

    m_io = io;
    m_inputs = new SwerveModuleInputsAutoLogged();

    m_desiredState = getState();
  }

  public void update() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs(m_name, m_inputs);

    // Apply voltages
    m_io.setTargetTurnPosition(m_desiredState.angle);
    m_io.setTargetDriveVelocity(m_desiredState.speedMetersPerSecond);
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
