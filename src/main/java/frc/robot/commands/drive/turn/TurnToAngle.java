package frc.robot.commands.drive.turn;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;

public class TurnToAngle extends BaseTurnCommand {
    protected final Rotation2d m_desiredRobotAngle;

    public TurnToAngle(Drive drive, Rotation2d angle) {
        super(drive);
        m_desiredRobotAngle = angle;
    }

    /**
     * @return The desired angle in radians
     */
    public Rotation2d getDesiredAngle() {
        return m_desiredRobotAngle;
    }

}
