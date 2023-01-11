package frc.lib.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

public class FaceTargetDrive extends TargetAngleDrive {
    private static final double kPredictiveMultiplier = 5.5;
    private final Supplier<Translation2d> m_targetSupplier;

    public FaceTargetDrive(Drive drive, Supplier<Translation2d> targetSupplier) {
        super(drive);
        m_targetSupplier = targetSupplier;
    }

    public FaceTargetDrive(Drive drive, Translation2d target) {
        this(drive, () -> target);
    }

    public FaceTargetDrive(Drive drive, Pose2d target) {
        this(drive, () -> target.getTranslation());
    }

    @Override
    protected Rotation2d getDesiredAngle() {
        Pose2d robotPose = getPredictiveRobotPose();
        Translation2d target = m_targetSupplier.get();

        Translation2d offset = target.minus(robotPose.getTranslation());

        return new Rotation2d(offset.getX(), offset.getY());
    }

    protected Pose2d getPredictiveRobotPose() {
        var chassisSpeeds = m_drive.getChassis().getChassisSpeeds();
        double deltaX = chassisSpeeds.vxMetersPerSecond * 0.02 * kPredictiveMultiplier;
        double deltaY = chassisSpeeds.vyMetersPerSecond * 0.02 * kPredictiveMultiplier;
        double deltaTheta = chassisSpeeds.omegaRadiansPerSecond * 0.02 * kPredictiveMultiplier;

        return RobotState.getInstance().getRobotPose().exp(new Twist2d(deltaX, deltaY, deltaTheta));
    }

}
