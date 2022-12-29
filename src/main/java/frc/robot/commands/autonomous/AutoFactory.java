package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;

public class AutoFactory {
    public static final PIDConstants linearPIDConstants = new PIDConstants(1, 0, 0);
    public static final PIDConstants angularPIDConstants = new PIDConstants(1, 0, 0);

    private final Drive m_drive;
    private final Camera m_camera;

    private final HashMap<String, Command> m_eventMap;

    private String m_selectedAutoName;
    private ArrayList<PathPlannerTrajectory> m_paths;

    public AutoFactory(Drive drive, Camera camera) {
        m_drive = drive;
        m_camera = camera;

        // Define PathPlanner Event Map
        m_eventMap = new HashMap<String, Command>(Map.of(
                "a", Commands.print("a"), // First Shoot Location
                "b", Commands.print("b"), // Second Shoot Location
                "c", Commands.print("c"), // Loading Cargo
                "d", Commands.print("d") // Third Shoot Location
        ));
    }

    public void loadAutoPathByName(String name) {
        m_selectedAutoName = name;
        loadSelectedPathFromFile();
    }

    public void loadSelectedPathFromFile() {
        if (m_selectedAutoName == null || m_selectedAutoName.isBlank()) {
            throw new RuntimeException("Must call 'loadAutoPathByName()' first!");
        }

        m_paths = PathPlanner.loadPathGroup(
                m_selectedAutoName,
                DriveConstants.kAutoMaxSpeedMetersPerSecond,
                DriveConstants.kAutoMaxAccelerationMetersPerSecondSq);
    }

    public CommandBase getAutoCommand() {
        if (m_paths == null) {
            loadAutoPathByName(AutoConstants.kDefaultAuto);
        }

        // Create Auto builder
        BaseAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_drive::getPose,
                m_drive::resetPose,
                m_drive.getChassis().getKinematics(),
                linearPIDConstants, angularPIDConstants,
                m_drive::setModuleStates,
                m_eventMap,
                m_drive);

        return autoBuilder.fullAuto(m_paths).andThen(m_drive.brakeCommand());
    }
}
