package frc.robot.commands.debug;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.commands.drive.BaseDriveCommand;
import frc.lib.commands.drive.DriveCommandWrapper;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.FieldUtil;

public final class DriveBoundary extends DriveCommandWrapper {

    public static class Bounds {
        public static final Bounds kFieldBounds = new Bounds(
                FieldConstants.kFieldWidthMeters,
                FieldConstants.kFieldHeightMeters);

        public final double x;
        public final double y;
        public final double width;
        public final double height;
        public final double xMax;
        public final double yMax;

        public Bounds(double x, double y, double width, double height) {
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;

            this.xMax = x + width;
            this.yMax = y + height;
        }

        public Bounds(double width, double height) {
            this(0, 0, width, height);
        }

        public static Bounds fromTranslations(Translation2d bottomLeft, Translation2d topRight) {
            double x = bottomLeft.getX();
            double y = bottomLeft.getY();
            double width = topRight.getX() - bottomLeft.getX();
            double height = topRight.getY() - bottomLeft.getY();
            return new Bounds(x, y, width, height);
        }

        public Pose2d[] getCornerPoses() {
            return new Pose2d[] {
                    new Pose2d(x, y, Rotation2d.fromDegrees(0)),
                    new Pose2d(xMax, y, Rotation2d.fromDegrees(0)),
                    new Pose2d(xMax, yMax, Rotation2d.fromDegrees(0)),
                    new Pose2d(x, yMax, Rotation2d.fromDegrees(0))
            };
        }
    }

    private static final boolean kShowBoundsOnField = true;
    private static final String kDriveBoundsFieldObjectName = "DriveBoundary";
    private final Bounds m_bounds;

    public DriveBoundary(BaseDriveCommand driveCommand, Bounds bounds) {
        super(driveCommand);
        m_bounds = bounds;
    }

    public DriveBoundary(BaseDriveCommand driveCommand) {
        this(driveCommand, Bounds.kFieldBounds);
    }

    @Override
    public void initialize() {
        super.initialize();

        if (kShowBoundsOnField) {
            FieldUtil.getDefaultField().setObjectPoses(kDriveBoundsFieldObjectName, m_bounds.getCornerPoses());
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if (kShowBoundsOnField) {
            FieldUtil.getDefaultField().removeObject(kDriveBoundsFieldObjectName);
        }
    }

    @Override
    protected ChassisSpeeds getChassisSpeeds(double x, double y, double omega,
            boolean isFieldRelative) {
        ChassisSpeeds speeds = super.getChassisSpeeds(x, y, omega, isFieldRelative);
        Pose2d pose = m_drive.getPose();

        // Convert chassis speeds to field relative
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation().unaryMinus());

        if (headingOutOfBounds(speeds.vxMetersPerSecond, pose.getX(), m_bounds.x, m_bounds.xMax)) {
            speeds.vxMetersPerSecond = 0;
        }

        if (headingOutOfBounds(speeds.vyMetersPerSecond, pose.getY(), m_bounds.y, m_bounds.yMax)) {
            speeds.vyMetersPerSecond = 0;
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation());
    }

    private boolean headingOutOfBounds(double speed, double pos, double minPos, double maxPos) {
        return speed <= 0 && pos <= minPos || speed >= 0 && pos >= maxPos;
    }
}
