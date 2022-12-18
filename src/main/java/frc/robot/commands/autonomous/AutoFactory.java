package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.debug.DebugCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Camera;

public class AutoFactory {
    public static final PIDConstants linearPIDConstants = new PIDConstants(1, 0, 0);
    public static final PIDConstants angularPIDConstants = new PIDConstants(1, 0, 0);

    private final Drive m_drive;
    private final Camera m_camera;

    private final HashMap<String, Command> m_eventMap;

    private ArrayList<PathPlannerTrajectory> m_paths;

    public AutoFactory(Drive drive, Camera camera) {
        m_drive = drive;
        m_camera = camera;

        // Define PathPlanner Event Map
        m_eventMap = new HashMap<String, Command>(Map.of(
                "PreShoot1", DebugCommands.putSmartDashboardValue(AutoConstants.kAutoStatusKey, "Shooting 1"), // First Shoot Location
                "PreShoot2", DebugCommands.putSmartDashboardValue(AutoConstants.kAutoStatusKey, "Shooting 2"), // Second Shoot Location
                "PreLoadStation", DebugCommands.putSmartDashboardValue(AutoConstants.kAutoStatusKey, "Loading Cargo"), // Loading Cargo
                "PreShoot3", DebugCommands.putSmartDashboardValue(AutoConstants.kAutoStatusKey, "Shooting 3") // Third Shoot Location
        ));
    }

    public void loadAutoPathByName(String name) {
        m_paths = PathPlanner.loadPathGroup(
                name,
                DriveConstants.kAutoMaxSpeedMetersPerSecond,
                DriveConstants.kAutoMaxAccelerationMetersPerSecondSq);
    }

    public CommandBase getAutoCommand() {

        if (m_paths == null) {
            loadAutoPathByName(AutoConstants.kDefaultAuto);
        }

        // Create Auto builder
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_drive::getPose,
                m_drive::resetPose,
                m_drive.getChassis().getKinematics(),
                linearPIDConstants, angularPIDConstants,
                m_drive::setModuleStates,
                m_eventMap,
                m_drive);

        return autoBuilder.fullAuto(m_paths).andThen(() -> m_drive.brake(), m_drive);
    }
}
