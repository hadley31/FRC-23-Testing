package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PhotonSubsystem extends SubsystemBase {
  private final PhotonCamera m_camera;
  private final Transform3d m_robotToCamera;

  private PhotonPipelineResult m_latestResult;

  public PhotonSubsystem(String cameraName, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(cameraName);
    m_robotToCamera = robotToCamera;
  }

  @Override
  public final void periodic() {
    m_latestResult = getCamera().getLatestResult();
    periodic(m_latestResult);
  }

  public abstract void periodic(PhotonPipelineResult result);

  public PhotonCamera getCamera() {
    return m_camera;
  }

  public Transform3d getRobotToCamera() {
    return m_robotToCamera;
  }

  public Transform3d getCameraToRobot() {
    return m_robotToCamera.inverse();
  }

  public Pose3d getCameraPose(Pose3d robotPose) {
    return robotPose.transformBy(getRobotToCamera());
  }

  public PhotonPipelineResult getLatestResult() {
    return m_latestResult;
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    return Optional.ofNullable(getLatestResult()).map(PhotonPipelineResult::getBestTarget);
  }
}
