package frc.robot.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.drive.Drive;

public class FollowSuppliedTarget extends FollowTarget {
    private final Supplier<Optional<Pose2d>> m_poseSupplier;

    public FollowSuppliedTarget(Drive drive, Transform2d desiredOffset, Supplier<Optional<Pose2d>> poseSupplier) {
        super(drive, desiredOffset);
        m_poseSupplier = poseSupplier;
    }

    @Override
    protected Optional<Pose2d> getTargetPose() {
        return m_poseSupplier.get();
    }
}
