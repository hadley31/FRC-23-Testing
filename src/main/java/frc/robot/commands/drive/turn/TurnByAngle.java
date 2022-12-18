package frc.robot.commands.drive.turn;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;

public class TurnByAngle extends BaseTurnCommand {
    private Rotation2d m_inputTurnAngle;
    private Rotation2d m_desiredRobotAngle;

    public TurnByAngle(Drive drive, Rotation2d angle) {
        super(drive);
        m_inputTurnAngle = angle;
    }

    @Override
    public void initialize() {
        m_desiredRobotAngle = m_drive.getPose().getRotation().plus(m_inputTurnAngle);
    }

    @Override
    public Rotation2d getDesiredAngle() {
        return m_desiredRobotAngle;
    }

}
