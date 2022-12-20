package frc.robot.commands.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.FollowTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;
import frc.robot.util.GeometryUtils;

public class FollowVisionTarget extends FollowTarget {
    private final Camera m_camera;
    private final int m_fiducialId;

    public FollowVisionTarget(Drive drive, Camera camera, Transform2d desiredOffset, int fiducialId) {
        super(drive, desiredOffset);

        m_camera = camera;
        m_fiducialId = fiducialId;
    }

    @Override
    protected Optional<Pose2d> getTargetPose() {
        var latestResult = m_camera.getLatestResult();

        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }

        var bestTarget = latestResult.getBestTarget();

        if (bestTarget.getFiducialId() != m_fiducialId) {
            return Optional.empty();
        }

        var bestOffset = latestResult.getBestTarget().getBestCameraToTarget();

        var tagPose = GeometryUtils.from2dTo3d(m_drive.getPose()).transformBy(VisionConstants.kCameraToRobot.inverse())
                .transformBy(bestOffset);

        return Optional.of(tagPose.toPose2d());
    }
}
