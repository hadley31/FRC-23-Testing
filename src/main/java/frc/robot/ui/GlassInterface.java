package frc.robot.ui;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.chassis.DriveChassis;

public class GlassInterface {
    public static final String kFieldName = "Field";
    private static final Field2d field = new Field2d();

    public static void initialize() {
        SmartDashboard.putData(kFieldName, field);
    }

    public static void setObjects(Map<String, Pose2d> objects) {
        for (var entry : objects.entrySet()) {
            String objectName = entry.getKey();
            Pose2d objectPose = entry.getValue();
            field.getObject(objectName).setPose(objectPose);
        }
    }

    public static void setObjects3d(Map<String, Pose3d> objects) {
        for (var entry : objects.entrySet()) {
            String objectName = entry.getKey();
            Pose2d objectPose = entry.getValue().toPose2d();
            field.getObject(objectName).setPose(objectPose);
        }
    }

    public static void setObjectPose(String name, Pose2d pose) {
        field.getObject(name).setPose(pose);
    }

    public static void setObjectPoses(String name, Pose2d... poses) {
        field.getObject(name).setPoses(poses);
    }

    public static void setTrajectory(String name, Trajectory trajectory) {
        field.getObject(name).setTrajectory(trajectory);
    }

    public static void updateRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    public static void setSwerveRobotPose(Pose2d pose, DriveChassis chassis) {
        updateRobotPose(pose);

        var states = chassis.getModuleStates();
        var translations = chassis.getModuleTranslations();

        Pose2d[] modulePoses = new Pose2d[states.length];

        for (int i = 0; i < states.length; i++) {
            var translation = translations[i];
            var rotation = states[i].angle;

            modulePoses[i] = pose.transformBy(new Transform2d(translation, rotation));
        }

        setObjectPoses("RobotSwerveModules", modulePoses);
    }

    public static Pose2d getObjectPose(String name) {
        return field.getObject(name).getPose();
    }
}
