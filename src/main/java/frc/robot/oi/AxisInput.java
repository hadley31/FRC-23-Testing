package frc.robot.oi;

import edu.wpi.first.math.MathUtil;

public class AxisInput {
    private final double m_deadzone;

    public AxisInput(double deadzone) {
        m_deadzone = Math.abs(deadzone);
    }

    public double calculate(double input) {
        return MathUtil.applyDeadband(input, m_deadzone);
    }
}
