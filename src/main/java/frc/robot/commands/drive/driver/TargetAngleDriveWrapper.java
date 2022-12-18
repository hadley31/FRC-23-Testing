package frc.robot.commands.drive.driver;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TargetAngleDriveWrapper extends TargetAngleDrive {
    private final TargetAngleDrive m_targetAngleDrive;

    public TargetAngleDriveWrapper(TargetAngleDrive targetAngleDrive) {
        super(targetAngleDrive.m_drive, targetAngleDrive.m_xInputSupplier, targetAngleDrive.m_yInputSupplier);
        addRequirements(targetAngleDrive.getRequirements().toArray(Subsystem[]::new));
        m_targetAngleDrive = targetAngleDrive;
    }

    /**
     * Override Built-In Methods
     */

    @Override
    public void initialize() {
        m_targetAngleDrive.initialize();
    }

    @Override
    public boolean isFinished() {
        return m_targetAngleDrive.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_targetAngleDrive.end(interrupted);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return m_targetAngleDrive.getRequirements();
    }

    /**
     * Override BaseDriveCommand Methods
     */

    @Override
    protected double getXSpeed() {
        return m_targetAngleDrive.getXSpeed();
    }

    @Override
    protected double getYSpeed() {
        return m_targetAngleDrive.getYSpeed();
    }

    @Override
    protected boolean getFieldRelative() {
        return m_targetAngleDrive.getFieldRelative();
    }

    /**
     * Override TargetAngleDrive Method
     */

    @Override
    protected Rotation2d getDesiredAngle() {
        return m_targetAngleDrive.getDesiredAngle();
    }
}
