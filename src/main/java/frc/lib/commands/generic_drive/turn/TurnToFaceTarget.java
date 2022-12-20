package frc.lib.commands.generic_drive.turn;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.commands.generic_drive.DriveCommandConfig;

public class TurnToFaceTarget extends BaseTurnCommand {

    private Supplier<Translation2d> m_targetSupplier;

    public TurnToFaceTarget(DriveCommandConfig drive, PIDController controller,
            Supplier<Translation2d> targetSupplier) {
        super(drive, controller);
        m_targetSupplier = targetSupplier;
    }

    public TurnToFaceTarget(DriveCommandConfig drive, PIDController controller, Translation2d target) {
        this(drive, controller, () -> target);
    }

    public TurnToFaceTarget(DriveCommandConfig drive, PIDController controller, Pose2d target) {
        this(drive, controller, target.getTranslation());
    }

    @Override
    public Rotation2d getDesiredAngle() {
        Translation2d offset = m_targetSupplier.get().minus(m_drive.getPose().getTranslation());
        System.out.println(offset);
        return new Rotation2d(offset.getX(), offset.getY());
    }

}
