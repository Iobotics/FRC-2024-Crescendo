// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

public class Conversions {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }
    
    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }

    /**
     * @param counts REV Position Counts
     * @param gearRatio Gear Ratio between REV and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double sparkToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 42.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between REV and Mechanism
     * @return REV Position Counts
     */
    public static double degreesToSpark(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 42.0));
    }

    /**
     * @param velocityCounts REV Velocity Counts
     * @param gearRatio Gear Ratio between REV and Mechanism (set to 1 for REV RPM)
     * @return RPM of Mechanism
     */
    public static double sparkToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 42.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between REV and Mechanism (set to 1 for REV RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToSpark(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (42.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts REV Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between REV and Mechanism (set to 1 for REV MPS)
     * @return REV Velocity Counts
     */
    public static double sparkToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = sparkToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between REV and Mechanism (set to 1 for REV MPS)
     * @return REV Velocity Counts
     */
    public static double MPSToSpark(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToSpark(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts REV Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between REV and Wheel
     * @return Meters
     */
    public static double sparkToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * (circumference / (gearRatio * 42));
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between REV and Wheel
     * @return REV Position Counts
     */
    public static double MetersToSpark(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * 42));
    }

}