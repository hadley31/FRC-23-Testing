package frc.robot.util;

/**
 * Runs a function every N calls
 */
public class NotSoPeriodic {
    private final int m_ticks;
    private int m_calls;

    public NotSoPeriodic(int nTicks) {
        m_ticks = nTicks;
    }

    public void call(Runnable func) {
        if (++m_calls >= m_ticks) {
            func.run();
            m_calls = 0;
        }
    }
}
