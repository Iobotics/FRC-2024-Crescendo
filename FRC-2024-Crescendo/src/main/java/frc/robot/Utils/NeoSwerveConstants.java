// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class NeoSwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue angleMotorInvert;
    public final SensorDirectionValue cancoderInvert;

    public NeoSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, double angleKF, InvertedValue driveMotorInvert, InvertedValue angleMotorInvert, SensorDirectionValue canCoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = canCoderInvert;
    }

    /** Swerve Drive Specialties - MK4i Module*/
    public static NeoSwerveConstants SDSMK4i(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.01;
        double angleKF = 0.0;

        InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new NeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios{
        /* SDS MK4i */
        /** SDS MK4i - 8.14 : 1 */
        public static final double SDSMK4i_L1 = (8.14 / 1.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75 / 1.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double SDSMK4i_L3 = (6.12 / 1.0);
    }
}