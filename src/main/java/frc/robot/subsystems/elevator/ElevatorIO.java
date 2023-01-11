package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import frc.lib.advantagekit.LoggedIO;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public interface ElevatorIO extends LoggedIO<ElevatorIOInputs> {
    @AutoLog
    public static class ElevatorIOInputs {
        public double motorVoltage;
    }

    public void setHeightMeters(double heightMeters);

    public double getHeightMeters();
}
