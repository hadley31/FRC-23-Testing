package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;

public class AutoFactory {
    public static final PIDConstants linearPIDConstants = new PIDConstants(1, 0, 0);
    public static final PIDConstants angularPIDConstants = new PIDConstants(1, 0, 0);

    private final Drive m_drive;
    private final Camera m_camera;

    private final HashMap<String, Command> m_eventMap;

    private String m_selectedAutoName;
    private List<PathPlannerTrajectory> m_paths;

    public AutoFactory(Drive drive, Camera camera) {
        m_drive = drive;
        m_camera = camera;

        // Define PathPlanner Event Map
        m_eventMap = new HashMap<String, Command>(Map.of(
                "a", Commands.print("a"),
                "b", Commands.print("b"),
                "c", Commands.print("c"),
                "d", Commands.print("d")));
    }

    public void loadAutoPathByName(String name) {
        m_selectedAutoName = name;
        loadSelectedPathFromFile();
    }

    public void loadSelectedPathFromFile() {
        if (m_selectedAutoName == null || m_selectedAutoName.isBlank()) {
            DriverStation.reportWarning("Unable to load pathplanner path. No auto selected!", false);
            return;
        }

        System.out.println("Loading path: " + m_selectedAutoName);

        m_paths = PathPlanner.loadPathGroup(
                m_selectedAutoName,
                DriveConstants.kAutoMaxSpeedMetersPerSecond,
                DriveConstants.kAutoMaxAccelerationMetersPerSecondSq);

        if (m_paths == null) {
            DriverStation.reportError("Failed to load path!", false);
        }
    }

    public CommandBase getAutoCommand() {
        if (m_paths == null) {
            DriverStation.reportWarning("No auto path selected... Loading default auto.", false);
            loadAutoPathByName(AutoConstants.kDefaultAuto);

            if (m_paths == null) {
                DriverStation.reportError("Loading default auto failed. Doing nothing.", false);
                return Commands.none();
            }
        }

        // Create Auto builder
        BaseAutoBuilder autoBuilder = new SwerveAutoBuilder(
                RobotState.getInstance()::getRobotPose,
                RobotState.getInstance()::resetRobotPose,
                m_drive.getChassis().getKinematics(),
                linearPIDConstants, angularPIDConstants,
                m_drive::setModuleStates,
                m_eventMap,
                m_drive);

        return autoBuilder.fullAuto(m_paths).andThen(m_drive.brakeCommand());
    }
}
