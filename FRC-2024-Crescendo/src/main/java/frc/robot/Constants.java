// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.NeoSwerveConstants;
import frc.robot.Utils.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double stickDeadband = 0.2;

    public static final class OIConstants {
        public static int kGamepad = 0;
        public static int kJoystick1 = 1;
        public static int kJoystick2 = 2;
        public static int kFight = 3;
        public static int kSwifferGamepad = 4;
    }

    public static final class PIDConstants {
        public static final int kPIDPrimary = 0;
    }

    public static final class VisionConstants {
        public static final Pose2d redSpeaker = new Pose2d(16.5293, 5.4579, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d redAmp = new Pose2d(14.7008, 8.2042, new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d redSource = new Pose2d(0.9088,0.5648,   new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d redStageCenter = new Pose2d(11.2202, 4.1051, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d redStageLeft = new Pose2d(11.9047, 3.7132, new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d redStageRight = new Pose2d(11.9047, 4.4983, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d blueSpeaker = new Pose2d(-0.0381, 5.5479,   new Rotation2d(Math.toRadians(0)));
        public static final Pose2d blueAmp = new Pose2d(1.8415, 8.2042,  new Rotation2d(Math.toRadians(-90)));
        public static final Pose2d blueSource = new Pose2d(15.6323,0.5648,  new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d blueStageCenter = new Pose2d(5.3208, 4.1051, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d blueStageLeft = new Pose2d(4.6413, 4.4983, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d blueStageRight = new Pose2d(4.6413, 3.713, new Rotation2d(Math.toRadians(-120)));


        public static final String kFrontCameraName = "Swiffer_Camera";
        public static final String kIntakeCameraName = "Intake_Camera";
        public static final int COLORED_SHAPE_PIPELINE = 0;
        public static final int APRILTAG_PIPELINE = 1;
        public static final Transform3d kRobotToCam = 
                new Transform3d(new Translation3d(0.0, 0.2079244, 0.0), new Rotation3d(0, Math.toRadians(25), Math.toRadians(180)));
        // 0.2079244, 0.2940558
        public static final List<AprilTag> apriltags = Arrays.asList(
            new AprilTag(1, new Pose3d(new Translation3d(15.0795, 0.2459, 1.3559), new Rotation3d(0.0, 0.0, Math.toRadians(120.0)))),
            new AprilTag(2, new Pose3d(new Translation3d(16.1851, 0.8837, 1.3559), new Rotation3d(0.0, 0.0, Math.toRadians( 120.0)))),
            new AprilTag(3, new Pose3d(new Translation3d(16.5793, 4.9827, 1.4511), new Rotation3d(0.0, 0.0, Math.toRadians( 180.0)))),
            new AprilTag(4, new Pose3d(new Translation3d(16.5793, 5.5479, 1.4511), new Rotation3d(0.0, 0.0, Math.toRadians( 180.0)))),
            new AprilTag(5, new Pose3d(new Translation3d(14.7008, 8.2042, 1.3559), new Rotation3d(0.0, 0.0, Math.toRadians( -90.0)))),
            new AprilTag(6, new Pose3d(new Translation3d(1.8415, 8.2042, 1.3559), new Rotation3d(0.0, 0.0, Math.toRadians( -90.0)))),
            new AprilTag(7, new Pose3d(new Translation3d(-0.0381, 5.5479, 1.4511), new Rotation3d(0.0, 0.0, 0.0))),
            new AprilTag(8, new Pose3d(new Translation3d(-0.0381, 4.9827, 1.4511), new Rotation3d(0.0, 0.0, 0.0))),
            new AprilTag(9, new Pose3d(new Translation3d(0.3561, 0.8837, 1.3559), new Rotation3d(0.0, 0.0, Math.toRadians( 60.0)))),
            new AprilTag(10, new Pose3d(new Translation3d(1.4615, 0.2459, 1.3559), new Rotation3d(0.0, 0.0, Math.toRadians( 60.0)))),
            new AprilTag(11, new Pose3d(new Translation3d(11.9047, 3.7132, 1.3208), new Rotation3d(0.0, 0.0, Math.toRadians( -60.0)))),
            new AprilTag(12, new Pose3d(new Translation3d(11.9047, 4.4983, 1.3208), new Rotation3d(0.0, 0.0, Math.toRadians( 60.0)))),
            new AprilTag(13, new Pose3d(new Translation3d(11.2202, 4.1051, 1.3208), new Rotation3d(0.0, 0.0, Math.toRadians( 180.0)))),
            new AprilTag(14, new Pose3d(new Translation3d(5.3208, 4.1051, 1.3208), new Rotation3d(0.0, 0.0, 0.0))),
            new AprilTag(15, new Pose3d(new Translation3d(4.6413, 4.4983, 1.3208), new Rotation3d(0.0, 0.0, Math.toRadians( 120.0)))),
            new AprilTag(16, new Pose3d(new Translation3d(4.6413, 3.7132, 1.3208), new Rotation3d(0.0, 0.0, Math.toRadians( -120.0)))));
        public static final AprilTagFieldLayout k2024CrescendoTagField = new AprilTagFieldLayout(apriltags, 16.451, 8.211);


        public static final double translationKP = 0.8; // TODO: tuning
        public static final double translationKI = 0.0;
        public static final double translationKD = 0.0;

        public static final double strafeKP = 0.8;
        public static final double strafeKI = 0.0;
        public static final double strafeKD = 0.0;

        public static final double rotationKP = 0.01;
        public static final double rotationKI = 0.0;
        public static final double rotationKD = 0.0;

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */

        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
        .fill(
            0.1, // x
            0.1, // y
            0.2 * Math.PI // theta
        );

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
         * radians.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                1, // x
                1, // y
                1);
    }

    public static final class IntakeConstants{
        public static int kUI = 17;
        public static int kUS = 19;
        public static int kLI = 18;
        public static int kLS = 20;
        public static int kRA = 16;
        public static int kLA = 15;

        public static double kArmGearRatio = 12;
    }


    public static final class SwifferConstants{
        public static final int kArm = 23; //please change when ready
        public static final int kWrist = 25; //please change when ready
        public static final int kRoller = 24; //please change when ready

        public static final int kDigitalInput = 2; //please change when ready
    }

    public static final class ClimberConstants{
        public static final int kClimber1 = 21; //left
        public static final int kClimber2 = 22; //right 
        public static final int kCGearRatio = 18;
    }

    public static final class SwerveConstants {
        public static final int pigeonID = 14;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
        public static final int maxModuleSpeed = 4;

        public static final NeoSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                NeoSwerveConstants.SDSMK4i(NeoSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75); // TODO: This must be tuned to specific
                                                                              // robot
        public static final double wheelBase = Units.inchesToMeters(20.75); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 15;
        public static final int anglePeakCurrentLimit = 30;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 25;
        public static final int drivePeakCurrentLimit = 40;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /*
         * These values are used by the drive neo to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 2.0; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 2.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(19.86);
            public static final double CANoffsets = 0;
            public static final boolean invertedA = true;
            public static final boolean invertedD = true;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, CANoffsets, invertedA, invertedD);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(16.17);
            public static final double CANoffsets = 0;
            public static final boolean invertedA = true;
            public static final boolean invertedD = false;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, CANoffsets, invertedA, invertedD);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(132.80);
            public static final double CANoffsets = 0;
            public static final boolean invertedA = true;
            public static final boolean invertedD = true;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, CANoffsets, invertedA, invertedD);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(351.29);
            public static final double CANoffsets = 0;
            public static final boolean invertedA = true;
            public static final boolean invertedD = false;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset, CANoffsets, invertedA, invertedD);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;

        public static final double kPXController = 10;
        public static final double kPYController = 10;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}