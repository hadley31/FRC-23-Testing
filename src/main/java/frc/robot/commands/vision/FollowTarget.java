package frc.robot.commands.vision;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;
import frc.robot.ui.GlassInterface;
import frc.robot.util.GeometryUtils;

public class FollowTarget extends CommandBase {
    private final PIDController m_xController = new PIDController(6, 0, 0);
    private final PIDController m_yController = new PIDController(6, 0, 0);
    private final ProfiledPIDController m_thetaController = new ProfiledPIDController(6, 0, 0, new Constraints(
            DriveConstants.kAutoMaxSpeedMetersPerSecond, DriveConstants.kAutoMaxAccelerationMetersPerSecondSq));

    private final Drive m_drive;
    private final Camera m_camera;
    private final int m_pipelineIndex;
    private final Transform2d m_targetOffset;
    private final int m_targetFiducial;

    public FollowTarget(Drive drive, Camera camera, int pipelineIndex, int targetFiducial, Transform2d targetOffset) {
        m_drive = drive;
        m_camera = camera;
        m_pipelineIndex = pipelineIndex;
        m_targetFiducial = targetFiducial;
        m_targetOffset = targetOffset;

        addRequirements(drive, camera);
    }

    @Override
    public void initialize() {
        m_camera.setPipelineIndex(m_pipelineIndex, true);
        GlassInterface.setObjectPose("TestFiducial", FieldConstants.kHubPose.toPose2d());
    }

    @Override
    public void execute() {
        m_drive.drive(getSpeeds());
    }

    private ChassisSpeeds getSpeeds() {
        Optional<Pose2d> tagPose = getTestPose();

        SmartDashboard.putBoolean("FollowTarget/TagVisible", tagPose.isPresent());

        if (tagPose.isEmpty()) {
            return new ChassisSpeeds();
        }

        SmartDashboard.putNumber("FollowTarget/TagPoseX", tagPose.get().getX());
        SmartDashboard.putNumber("FollowTarget/TagPoseY", tagPose.get().getY());
        SmartDashboard.putNumber("FollowTarget/TagPoseDegrees", tagPose.get().getRotation().getDegrees());

        Pose2d desiredPose = tagPose.get().transformBy(m_targetOffset);

        var twist = m_drive.getPose().log(desiredPose);

        twist.dx = MathUtil.applyDeadband(twist.dx, 0.2, DriveConstants.kAutoMaxSpeedMetersPerSecond);
        twist.dy = MathUtil.applyDeadband(twist.dy, 0.6, DriveConstants.kAutoMaxSpeedMetersPerSecond);

        twist.dtheta = MathUtil.applyDeadband(twist.dtheta, Units.degreesToRadians(10),
                DriveConstants.kMaxSpeedRadiansPerSecond);

        return new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta);
    }

    private Optional<Pose2d> getCurrentTagPose() {
        var latestResult = m_camera.getLatestResult();

        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }

        var bestTarget = latestResult.getBestTarget();

        if (bestTarget.getFiducialId() != m_targetFiducial) {
            return Optional.empty();
        }

        var bestOffset = latestResult.getBestTarget().getBestCameraToTarget();

        var tagPose = GeometryUtils.from2dTo3d(m_drive.getPose()).transformBy(VisionConstants.kCameraToRobot.inverse())
                .transformBy(bestOffset);

        return Optional.of(tagPose.toPose2d());
    }

    public Optional<Pose2d> getTestPose() {
        return Optional.ofNullable(GlassInterface.getObjectPose("TestFiducial"));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.brake();
        m_camera.setLEDs(false);
    }
}
