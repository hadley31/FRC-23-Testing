package frc.robot.util;

import frc.robot.Constants;

public final class TalonFXUtils {

    public static final double kTicksPerRevolution = 2048;

    /**
     * Converts ticks per 100ms to revolutions per minute.
     * @param ticks ticks per 100ms
     * @return RPM
     */
    public static double ticksPer100msToRPM(double ticks) {
        return ticks * 600 / kTicksPerRevolution;
    }

    /**
     * Converts revolutions per minute ticks per 100ms.
     * @param rpm revolutions per minute
     * @return ticks per 100ms
     */
    public static double RPMToTicksPer100ms(double rpm) {
        return rpm * kTicksPerRevolution / 600;
    }

    public static double metersPerSecondToTicksPer100ms(double metersPerSecond, double scalingFactor) {
        return metersToTicks(metersPerSecond, scalingFactor) / 10;
    }

    public static double ticksPer100msToMetersPerSecond(double ticksPer100ms, double scalingFactor) {
        return ticksToMeters(ticksPer100ms, scalingFactor) * 10;
    }

    public static double metersToTicks(double meters, double gearRatio, double wheelDiameter) {
        return meters * kTicksPerRevolution * gearRatio / (Math.PI * wheelDiameter);
    }

    public static double ticksToMeters(double ticks, double gearRatio, double wheelDiameter) {
        return (ticks / kTicksPerRevolution) * (Math.PI * wheelDiameter) * (1 / gearRatio);
    }

    /**
    * Use this if you already have the scalingFactor pre-calculated
    * @param meters Distance
    * @param scalingFactor PI * wheelDiameter / gearRatio
    * @return
    */
    public static double metersToTicks(double meters, double scalingFactor) {
        return meters * kTicksPerRevolution / scalingFactor;
    }

    /**
     * Use this if you already have the scalingFactor pre-calculated
     * @param ticks Motor encoder tick reading
     * @param scalingFactor PI * wheelDiameter / gearRatio
     * @return
     */
    public static double ticksToMeters(double ticks, double scalingFactor) {
        return (ticks / kTicksPerRevolution) * scalingFactor;
    }

    public static double radiansToTicks(double radians) {
        return radiansToTicks(radians, 1.0);
    }

    public static double ticksToRadians(double ticks) {
        return ticksToRadians(ticks, 1.0);
    }

    public static double degreesToTicks(double degrees) {
        return degreesToTicks(degrees, 1.0);
    }

    public static double ticksToDegrees(double ticks) {
        return ticksToDegrees(ticks, 1.0);
    }

    public static double radiansToTicks(double radians, double scalingFactor) {
        return radians / Constants.TWO_PI * kTicksPerRevolution * scalingFactor;
    }

    public static double ticksToRadians(double ticks, double scalingFactor) {
        return (ticks / kTicksPerRevolution) * Constants.TWO_PI / scalingFactor;
    }

    public static double degreesToTicks(double degrees, double scalingFactor) {
        return degrees / 360.0 * kTicksPerRevolution * scalingFactor;
    }

    public static double ticksToDegrees(double ticks, double scalingFactor) {
        return (ticks / kTicksPerRevolution) * 360.0 / scalingFactor;
    }
}
