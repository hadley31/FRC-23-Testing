package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

/**
 * Work in progress. Want to test simulation. Unsuccessful so far
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private final FlywheelSim m_turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);
  private final FlywheelSim m_driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);

  private double m_turnRelativePositionRad = 0.0;
  private double m_turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;

  private double m_turnAppliedVolts = 0.0;
  private double m_driveAppliedVolts = 0.0;

  public SwerveModuleIOSim() {

  }

  /**
   * This overload exists only for compatibility with other IO classes, and for "hot swaps"
   * @param turnPort
   * @param drivePort
   * @param cancoderPort
   */
  public SwerveModuleIOSim(int turnPort, int drivePort, int cancoderPort) {
    this();
  }

  @Override
  public void updateInputs(SwerveModuleInputs inputs) {
    m_turnSim.update(Constants.loopPeriodSecs);
    m_driveSim.update(Constants.loopPeriodSecs);

    double angleDiffRad = m_turnSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs;
    m_turnRelativePositionRad = MathUtil.angleModulus(m_turnRelativePositionRad + angleDiffRad);

    inputs.turnAbsolutePositionRad = m_turnAbsolutePositionRad;
    inputs.turnPositionRad = m_turnRelativePositionRad;
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());
    inputs.turnTempCelcius = 0;

    inputs.drivePositionMeters = inputs.drivePositionMeters
        + (m_driveSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.driveVelocityMetersPerSec = m_driveSim.getAngularVelocityRPM()
        * DriveConstants.kDriveVelocityConversionFactor;
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps());
    inputs.driveTempCelcius = 0;
  }

  @Override
  public void setTurnVoltage(double volts) {
    m_turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_turnSim.setInputVoltage(m_turnAppliedVolts);
  }

  @Override
  public void setDriveVoltage(double volts) {
    m_driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    m_driveSim.setInputVoltage(m_driveAppliedVolts);
  }

  @Override
  public void zeroTurnEncoder() {
    // TODO Auto-generated method stub
  }

  @Override
  public void zeroDriveEncoder() {
    // TODO Auto-generated method stub

  }

  @Override
  public void syncTurnEncoderWithAbsolute() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setTurnBrakeMode(boolean coast) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setDriveBrakeMode(boolean brake) {
    // TODO Auto-generated method stub

  }
}
