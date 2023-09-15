package frc.robot.subsystems.vision;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;

public class ReflectiveTapeCamera extends PhotonSubsystem {

  public ReflectiveTapeCamera(String cameraName, Transform3d robotToCamera) {
    super(cameraName, robotToCamera);
  }

  @Override
  public void periodic(PhotonPipelineResult result) {
    
  }
}
