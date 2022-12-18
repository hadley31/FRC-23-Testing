package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.vision.Camera;

public class SetLEDState extends CommandBase {
    private final Camera m_camera;
    private final boolean m_enabled;

    public SetLEDState(Camera camera, boolean enabled) {
        m_camera = camera;
        m_enabled = enabled;
    }

    @Override
    public void execute() {
        m_camera.setLEDs(m_enabled);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
