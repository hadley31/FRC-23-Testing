package frc.lib.commands.generic_drive;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveCommandConfig {
    private final Supplier<Pose2d> m_poseSupplier;
    private final Consumer<ChassisSpeeds> m_speedsConsumer;
    private final Subsystem[] m_requirements;

    public DriveCommandConfig(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> speedsConsumer,
            Subsystem... requirements) {
        m_poseSupplier = requireNonNullParam(poseSupplier, "poseSupplier", "DriveCommandConfig");
        m_speedsConsumer = requireNonNullParam(speedsConsumer, "speedsConsumer", "DriveCommandConfig");
        m_requirements = requirements;
    }

    public Pose2d getPose() {
        return m_poseSupplier.get();
    }

    public void acceptSpeeds(ChassisSpeeds speeds) {
        m_speedsConsumer.accept(speeds);
    }

    public Subsystem[] getRequirements() {
        return this.m_requirements;
    }
}
