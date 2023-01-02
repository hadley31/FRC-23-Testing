package frc.robot.commands.drive;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;

public abstract class FollowTarget extends CommandBase {
    protected final Drive m_drive;
    protected final Transform2d m_desiredOffset;

    public FollowTarget(Drive drive, Transform2d desiredOffset) {
        m_drive = drive;
        m_desiredOffset = desiredOffset;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        m_drive.drive(getSpeeds());
    }

    private ChassisSpeeds getSpeeds() {
        Optional<Pose2d> targetPose = getTargetPose();

        SmartDashboard.putBoolean("FollowTarget/IsVisible", targetPose.isPresent());

        if (targetPose.isEmpty()) {
            return new ChassisSpeeds();
        }

        SmartDashboard.putNumber("FollowTarget/PoseX", targetPose.get().getX());
        SmartDashboard.putNumber("FollowTarget/PoseY", targetPose.get().getY());
        SmartDashboard.putNumber("FollowTarget/PoseDegrees", targetPose.get().getRotation().getDegrees());

        Pose2d desiredPose = targetPose.get().transformBy(m_desiredOffset);

        var twist = m_drive.getPose().log(desiredPose);

        twist.dx = MathUtil.applyDeadband(twist.dx, 0.2, DriveConstants.kAutoMaxSpeedMetersPerSecond);
        twist.dy = MathUtil.applyDeadband(twist.dy, 0.6, DriveConstants.kAutoMaxSpeedMetersPerSecond);

        twist.dtheta = MathUtil.applyDeadband(twist.dtheta, Units.degreesToRadians(10),
                DriveConstants.kMaxSpeedRadiansPerSecond);

        return new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta);
    }

    protected abstract Optional<Pose2d> getTargetPose();

    @Override
    public void end(boolean interrupted) {
        m_drive.brake();
    }
}
