package frc.robot.commands.drive.turn;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.Drive;

public class TurnToFaceTarget extends BaseTurnCommand {

    private Supplier<Translation2d> m_targetSupplier;

    public TurnToFaceTarget(Drive drive, Supplier<Translation2d> targetSupplier) {
        super(drive);
        m_targetSupplier = targetSupplier;
    }

    public TurnToFaceTarget(Drive drive, Translation2d target) {
        this(drive, () -> target);
    }

    public TurnToFaceTarget(Drive drive, Pose2d target) {
        this(drive, target.getTranslation());
    }

    @Override
    public Rotation2d getDesiredAngle() {
        Translation2d offset = m_targetSupplier.get().minus(m_drive.getPose().getTranslation());
        System.out.println(offset);
        return new Rotation2d(offset.getX(), offset.getY());
    }

}
