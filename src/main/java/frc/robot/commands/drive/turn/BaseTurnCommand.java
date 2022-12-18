package frc.robot.commands.drive.turn;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public abstract class BaseTurnCommand extends CommandBase {
    private static final double kMaxErrorRadians = Units.degreesToRadians(1.0);
    private static final double kMaxErrorRadiansPerSecond = Units.degreesToRadians(1.0);

    protected final Drive m_drive;

    // TODO: tune PID
    protected PIDController m_controller = new PIDController(3, 0, 0.1);

    public BaseTurnCommand(Drive drive) {
        m_drive = drive;

        m_controller.enableContinuousInput(-Math.PI, Math.PI);
        m_controller.setTolerance(kMaxErrorRadians, kMaxErrorRadiansPerSecond);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double currentAngle = m_drive.getPose().getRotation().getRadians();
        double desiredAngleSetpoint = getDesiredAngle().getRadians();

        double rotationSpeed = m_controller.calculate(currentAngle, desiredAngleSetpoint);

        m_drive.drive(new ChassisSpeeds(0, 0, rotationSpeed));
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.brake();
    }

    /**
     * @return The desired robot rotation
     */
    public abstract Rotation2d getDesiredAngle();
}
