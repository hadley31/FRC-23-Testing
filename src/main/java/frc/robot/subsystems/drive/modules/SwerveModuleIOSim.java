package frc.robot.subsystems.drive.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Work in progress. Want to test simulation. Unsuccessful so far
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
    private SwerveModuleState m_currentState;
    private SwerveModuleState m_desiredState;
    private SwerveModulePosition m_currentPosition;

    public SwerveModuleIOSim() {
        m_currentState = new SwerveModuleState();
        m_desiredState = new SwerveModuleState();
        m_currentPosition = new SwerveModulePosition();
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        // Update internal state
        double oldAngleRads = m_currentPosition.angle.getRadians();

        double updatedAngle = MathUtil.interpolate(m_currentState.angle.getRadians(), m_desiredState.angle.getRadians(),
                0.9);
        m_currentState.angle = Rotation2d.fromRadians(updatedAngle);
        m_currentState.speedMetersPerSecond = MathUtil.interpolate(m_currentState.speedMetersPerSecond,
                m_desiredState.speedMetersPerSecond, 0.9);

        m_currentPosition.angle = m_currentState.angle;
        m_currentPosition.distanceMeters += m_currentState.speedMetersPerSecond * 0.02;

        // Update inputs
        inputs.driveAppliedVolts = 0;
        inputs.drivePositionMeters = m_currentPosition.distanceMeters;
        inputs.driveVelocityMetersPerSec = m_currentState.speedMetersPerSecond;

        inputs.turnAppliedVolts = 0;
        inputs.turnAbsolutePositionRad = getAbsoluteRotation().getRadians();
        inputs.turnPositionRad = getRotation().getRadians();
        inputs.turnVelocityRadPerSec = (m_currentPosition.angle.getRadians() - oldAngleRads) / Constants.kLoopTime;
    }

    @Override
    public SwerveModuleState getState() {
        return m_currentState;
    }

    @Override
    public SwerveModulePosition getPosition() {
        return m_currentPosition;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        var optimized = SwerveModuleState.optimize(desiredState, m_currentState.angle);

        m_desiredState = optimized;
    }
}
