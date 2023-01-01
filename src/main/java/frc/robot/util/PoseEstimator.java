package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.vision.photonvision.RobotPoseEstimator;
import frc.lib.vision.photonvision.RobotPoseEstimator.PoseStrategy;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.chassis.DriveChassis;
import frc.robot.subsystems.vision.Camera;

public class PoseEstimator {
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final RobotPoseEstimator m_photonPoseEstimator;
    private final Camera m_camera;
    private final AprilTagFieldLayout m_tagLayout;

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to "trust" the
    // estimate from that particular component more than the others. 
    // This in turn means the particular component will have a stronger
    // influence on the final pose estimate.

    public PoseEstimator(DriveChassis chassis, Camera camera, AprilTagFieldLayout tagLayout) {
        m_poseEstimator = new SwerveDrivePoseEstimator(
                chassis.getKinematics(),
                chassis.getGyroAngle(),
                chassis.getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        var aprilTagPoseMap = GeometryUtils.aprilTagPoseMap(tagLayout);
        var cameras = List.of(Pair.of(camera.getPhotonCamera(), VisionConstants.kCameraToRobot));

        m_photonPoseEstimator = new RobotPoseEstimator(aprilTagPoseMap, PoseStrategy.CLOSEST_TO_LAST_POSE, cameras);
        m_camera = camera;
        m_tagLayout = tagLayout;
    }

    public void update(Rotation2d gyroAngle, SwerveModuleState[] moduleStates, SwerveModulePosition[] modulePositions) {
        m_poseEstimator.update(gyroAngle, modulePositions);
        addCameraMeasurement();
    }

    public Pose2d getEstimatedPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        m_poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
        m_photonPoseEstimator.setLastPose(new Pose3d(pose));
    }

    private void addCameraMeasurement() {
        var result = m_photonPoseEstimator.update();

        Pose2d estimatedRobotPose = result.getFirst().toPose2d();
        double latency = result.getSecond();

        if (latency == 0) {
            System.out.println("No target");
            return;
        }

        double timestamp = Timer.getFPGATimestamp() - latency;

        Logger.getInstance().recordOutput("VisionEstRobotPose", estimatedRobotPose);

        m_poseEstimator.addVisionMeasurement(estimatedRobotPose, timestamp);
    }

    private void old_addCameraMeasurement() {
        var latestResult = m_camera.getLatestResult();

        if (!latestResult.hasTargets()) {
            // We found no targets so don't add any pose estimate
            return;
        }

        // Calculate when the image was taken using latency
        double imageCaptureTime = Timer.getFPGATimestamp() - latestResult.getLatencyMillis();

        var bestTarget = latestResult.getBestTarget();

        int fiducialId = bestTarget.getFiducialId();

        Optional<Pose3d> targetPose = m_tagLayout.getTagPose(fiducialId);

        if (targetPose.isEmpty()) {
            // We found a target and we didn't store where it is, so don't add any pose estimate
            return;
        }

        // Calculate position of camera on field based on observed target position
        Transform3d camToTargetTransform = bestTarget.getBestCameraToTarget();
        Pose3d estCameraPose = targetPose.get().transformBy(camToTargetTransform.inverse());

        // Calculate position of robot on field based on calculated camera position
        Pose2d visionEstimatedRobotPose = estCameraPose.transformBy(VisionConstants.kCameraToRobot).toPose2d();

        // Update pose estimator with vision result
        m_poseEstimator.addVisionMeasurement(visionEstimatedRobotPose, imageCaptureTime);

        FieldUtil.getDefaultField().setObjectPose("CameraEstimatedPose", visionEstimatedRobotPose);
        Logger.getInstance().recordOutput("VisionPose", visionEstimatedRobotPose);
    }
}
