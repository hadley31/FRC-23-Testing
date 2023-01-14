package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Camera;
import frc.robot.util.FieldUtil;
import frc.robot.util.PoseEstimator;

public class RobotState {
    //#region Singleton
    private static RobotState kInstance;

    public static RobotState getInstance() {
        if (kInstance == null) {
            throw new RuntimeException("Robot state must be initialized!");
        }

        return kInstance;
    }

    public static RobotState initialize(Drive drive, Camera camera, Elevator elevator, AprilTagFieldLayout tagLayout) {
        kInstance = new RobotState(drive, camera, elevator, tagLayout);
        return kInstance;
    }
    //#endregion

    private final Drive m_drive;
    private final Camera m_camera;
    private final Elevator m_elevator;

    private final PoseEstimator m_poseEstimator;

    private RobotState(Drive drive, Camera camera, Elevator elevator, AprilTagFieldLayout tagLayout) {
        m_drive = drive;
        m_camera = camera;
        m_elevator = elevator;

        m_poseEstimator = new PoseEstimator(drive.getChassis(), camera, tagLayout);
    }

    public void periodic() {
        m_poseEstimator.update(m_drive.getHeading(), m_drive.getModulePositions());
        log();
    }

    public void simulationPeriodic() {

    }

    public Pose2d getRobotPose() {
        return m_poseEstimator.getEstimatedPose();
    }

    public Pose3d getRobotPose3d() {
        return new Pose3d(getRobotPose());
    }

    public void resetRobotPose(Pose2d pose) {
        Rotation2d offset = pose.getRotation();

        m_drive.getChassis().getGyro().reset(offset);
        m_poseEstimator.resetPose(pose, offset, m_drive.getModulePositions());
    }

    public Pose3d getClawPose() {
        var clawTransform = new Transform3d(
                new Translation3d(0, m_elevator.getHeightMeters(), 0),
                new Rotation3d());
        return getRobotPose3d().plus(clawTransform);
    }

    private void log() {
        // Log robot position
        Logger.getInstance().recordOutput("Odometry", getRobotPose());

        // Log camera position
        var cameraPose = getRobotPose3d().transformBy(VisionConstants.kCameraToRobot.inverse());
        Logger.getInstance().recordOutput("CameraPose", cameraPose);

        // Log robot 3d orientation
        Logger.getInstance().recordOutput("RobotOrientation",
                new Pose3d(6, 4, 1, m_drive.getChassis().getGyro().getOrientation()));

        FieldUtil.getDefaultField().setSwerveRobotPose(getRobotPose(), m_drive.getChassis());
    }
}
