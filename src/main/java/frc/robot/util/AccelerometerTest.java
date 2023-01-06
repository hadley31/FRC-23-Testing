package frc.robot.util;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.lib.accelerometer.Pigeon2Accel;

public class AccelerometerTest {
    private final BuiltInAccelerometer m_builtin;
    private final Accelerometer m_pigeon;

    private final List<Accelerometer> m_accelerometers;

    private LinearFilter m_filterX = LinearFilter.movingAverage(5);
    private LinearFilter m_filterY = LinearFilter.movingAverage(5);

    public AccelerometerTest() {
        m_builtin = new BuiltInAccelerometer();
        m_pigeon = new Pigeon2Accel(0);
        m_accelerometers = List.of(m_builtin, m_pigeon);
    }

    public void update() {
        double avgX = average(m_accelerometers.stream().map(x -> x.getX()));
        double avgY = average(m_accelerometers.stream().map(x -> x.getY()));

        m_filterX.calculate(avgX);
        m_filterY.calculate(avgY);
    }

    private double average(Stream<Double> values) {
        return values.collect(Collectors.averagingDouble(x -> x));
    }
}
