package frc.robot.commands.autonomous;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.EventManager;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.AprilTagCamera;

public class AutoFactory extends CommandBase {
  public static final PIDConstants linearPIDConstants = new PIDConstants(6, 0, 0);
  public static final PIDConstants angularPIDConstants = new PIDConstants(2, 0, 0);

  private final Drive m_drive;
  private final AprilTagCamera m_camera;
  private final Elevator m_elevator;

  private final Map<String, Command> m_eventMap;

  public AutoFactory(Drive drive, AprilTagCamera camera, Elevator elevator) {
    m_drive = drive;
    m_camera = camera;
    m_elevator = elevator;

    // Define PathPlanner Event Map
    m_eventMap = Map.of(
        "a", Commands.print("a"),
        "b", Commands.print("b"),
        "c", Commands.print("c"),
        "d", Commands.print("d"),
        "brake", m_drive.brakeCommand(),
        "elevatorLow", m_elevator.setPositionCommand(Units.feetToMeters(1.5)),
        "elevatorHigh", m_elevator.setPositionCommand(Units.feetToMeters(5)),
        "charge", ChargeStationBalance.charge(drive));

    if (!Constants.kDebugMode) {
      PPLibTelemetry.enableCompetitionMode();
    }

    var config = new HolonomicPathFollowerConfig(linearPIDConstants, angularPIDConstants,
        DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kTrackWidth / 2);

    AutoBuilder.configureHolonomic(drive::getPose, drive::resetPose, drive::getChassisSpeeds, drive::drive, config,
        drive);

    EventManager.registerCommands(m_eventMap);

    AutoBuilder.buildAuto("test");
  }

  public Command getAutoCommand(String name) {
    return AutoBuilder.buildAuto(name);
  }
}
