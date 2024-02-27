// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.Conversions;
import frc.robot.Utils.SwerveModuleInterface;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator poseEstimator;
    public Vision vision;
    public PIDController rotationController;
    public Swerve() {
        rotationController = new PIDController(1.0, 0.0, 0.0);
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        
        //zeroHeading();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(3.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions(), getPose(),
                VisionConstants.STATE_STANDARD_DEVIATIONS,
                VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS);
    }

    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getEstPose, // Robot pose supplier
            this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
                    Constants.SwerveConstants.maxSpeed / 4, // Max module speed, in m/s
                    0.467, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );
    }
    // for path planner
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getGyroYaw()
                                );
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? speeds
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);
        SmartDashboard.putNumber("desiredrot",speeds.omegaRadiansPerSecond);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    


    // test for pathplanner
    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds) {
        // SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        // SmartDashboard.putNumber("desiredrotpre",desiredChassisSpeeds.omegaRadiansPerSecond);
        // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed / 4);
        // SmartDashboard.putNumber("desiredrot",desiredChassisSpeeds.omegaRadiansPerSecond);

        // for(SwerveModule mod : mSwerveMods){
        //     mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        // }
        desiredChassisSpeeds.omegaRadiansPerSecond *= SwerveConstants.maxAngularVelocity;

        setModuleStates(Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds));
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed / 4);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }    

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleInterface mod : mSwerveMods){
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
  

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstPose() {
        SmartDashboard.putNumber("estrot",poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(getGyroYaw(),getModulePositions(),newPose);
    } 

    public void resetPose() {
        setCurrentPose(new Pose2d());
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    // public void zeroHeading(){
    //     swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    // }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public double getEstYaw() {
        return getEstPose().getRotation().getDegrees();
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void setGyro(double newValue){
        gyro.setYaw(newValue);
    }

    public void resetModulesToAbsolute(){
        for(SwerveModuleInterface mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Pose2d getPoseToGoal(double goalX, double goalY) {
        Pose2d goalPose = new Pose2d(goalX, goalY, new Rotation2d());
        Pose2d currentPose = this.getEstPose();
        double goalRotation = Math.toDegrees(Math.atan((currentPose.getY()-goalPose.getY())/(currentPose.getX()-goalPose.getX())));
        SmartDashboard.putNumber("goalrot", goalRotation);
        SmartDashboard.putNumber("goalX", currentPose.minus(goalPose).getX());
        SmartDashboard.putNumber("goalY", currentPose.minus(goalPose).getY());
        return new Pose2d(currentPose.minus(goalPose).getTranslation(),new Rotation2d(goalRotation));
    }
    public void getPoseToGoal(Pose2d goalPose) {
        getPoseToGoal(goalPose.getX(),goalPose.getY());
    }

    public double rotateToSpeaker() {
        Pose2d poseToGoal = getPoseToGoal(16.5793, 5.5479);
        return rotationController.calculate(getEstYaw(),poseToGoal.getRotation().getDegrees())/180;
    }

    @Override
    public void periodic(){
        getPoseToGoal(16.5793, 5.5479);
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        poseEstimator.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("X",getEstPose().getX());
        SmartDashboard.putNumber("Y",getEstPose().getY());
        SmartDashboard.putNumber("Rotation",getEstPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Gyro", MathUtil.inputModulus(getGyroYaw().getDegrees(),-180,180));

        // for(SwerveModuleInterface mod : mSwerveMods){
        //     var moduleNumber = mod.getModuleNumber();
        //     SmartDashboard.putNumber("Mod " + moduleNumber + " CANcoder", mod.getCANcoder().getDegrees()*360);
        //     SmartDashboard.putNumber("Mod " + moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        //     SmartDashboard.putNumber("Mod " + moduleNumber + " Absolute", Conversions.sparkToDegrees(mod.getAbsolutePosition(), Constants.SwerveConstants.angleGearRatio));
        // }
    }
}