package frc.robot.commands.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.FollowTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;

public class FollowVisionTarget extends FollowTarget {
    private final Camera m_camera;
    private final Optional<Integer> m_fiducialId;

    private FollowVisionTarget(Drive drive, Camera camera, Transform2d desiredOffset, Optional<Integer> fiducialId) {
        super(drive, desiredOffset);

        m_camera = camera;
        m_fiducialId = fiducialId;
    }

    public FollowVisionTarget(Drive drive, Camera camera, Transform2d desiredOffset) {
        this(drive, camera, desiredOffset, Optional.empty());
    }

    public FollowVisionTarget(Drive drive, Camera camera, Transform2d desiredOffset, int fiducialId) {
        this(drive, camera, desiredOffset, Optional.of(fiducialId));
    }

    @Override
    protected Optional<Pose2d> getTargetPose() {
        var latestResult = m_camera.getLatestResult();

        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }

        var bestTarget = latestResult.getBestTarget();

        if (m_fiducialId.isPresent() && bestTarget.getFiducialId() != m_fiducialId.get()) {
            return Optional.empty();
        }

        var bestOffset = latestResult.getBestTarget().getBestCameraToTarget();

        var targetPose = m_drive.getPose3d()
                .transformBy(VisionConstants.kCameraToRobot.inverse()) // to camera's pose
                .transformBy(bestOffset); // to target's pose

        return Optional.of(targetPose.toPose2d());
    }
}
