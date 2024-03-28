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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private final Field2d m_field = new Field2d();
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator poseEstimator;
    public Vision vision;
    public Swerve() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID, "Carnivore");
        
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
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(13.5, 5.5, new Rotation2d(Math.PI)));
    }

    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getEstPose, // Robot pose supplier
            this::setCurrentEstPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(3.0, 0.0, 0.0), // Rotation PID constants
                    4.0, // Max module speed, in m/s
                    0.37268062902, // Drive base radius in meters. Distance from robot center to furthest module.
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
        setModuleStates(Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02)));
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

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }
    
    public void setGyro(double newValue){
        gyro.setYaw(newValue);
    }

    /** calls pose estimator for latest position
     * @return Pose2d in meters*/
    public Pose2d getEstPose() {
        SmartDashboard.putNumber("estrot",poseEstimator.getEstimatedPosition().getRotation().getDegrees());
        return poseEstimator.getEstimatedPosition();
    }

    public void setCurrentEstPose(Pose2d newPose) {
        poseEstimator.resetPosition(getGyroYaw(),getModulePositions(),newPose);
    }

    public void resetEstPose() {
        setCurrentEstPose(new Pose2d());
    }

    // public void zeroHeading(){
    //     swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    // }

    /** @return estimated heading in deg -180 to 180 */
    public double getEstYaw() {
        double rotation = MathUtil.inputModulus(getEstPose().getRotation().getDegrees(),-180,180);
        return rotation;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModuleInterface mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    /** field relative pose from bot to goal, bot position taken from pose estimator
     * @param goalPose field pose of goal
     * @param returnParallelAngle return a rotation paralell to given goal, if false rotation is heading needed to look at goal
     * @return field relative pose to goal and rotation
     */ 
    public Pose2d getPoseToGoal(Pose2d goalPose, boolean returnParallelAngle) {
        Pose2d currentPose = this.getEstPose();
        double goalRotation = goalPose.getRotation().getRadians();
        if (!returnParallelAngle) {
            // arctan(y1-y2/x1-x2)
            goalRotation = MathUtil.inputModulus(Math.atan((currentPose.getY()-goalPose.getY())/(currentPose.getX()-goalPose.getX()))+Math.PI,-Math.PI,Math.PI);
        }
        SmartDashboard.putNumber("goalrot", Math.toDegrees(goalRotation));
        SmartDashboard.putNumber("goalX", currentPose.minus(goalPose).getX());
        SmartDashboard.putNumber("goalY", currentPose.minus(goalPose).getY());
        return new Pose2d(currentPose.minus(goalPose).getTranslation(),new Rotation2d(goalRotation));
    }
    /**
     * @param goalX x value of goal 
     * @param goalY y value of goal 
     * @return
     */
    public Pose2d getPoseToGoal(double goalX, double goalY) {
        return getPoseToGoal(new Pose2d(goalX,goalY, new Rotation2d()), false);
    }

    public Pose2d getPoseToSpeaker() {
        return getPoseToGoal(Constants.VisionConstants.redSpeaker,false);
    }
    
    public double getRotationToSpeaker() {
        return getPoseToSpeaker().getRotation().getDegrees();
    }

    /**move this to diff subsystem later */
    public double getShootingAngle() {
        double shootingAngle = 0;
        Pose2d poseToSpeaker = getPoseToSpeaker();
        double distanceToSpeaker = Math.hypot(poseToSpeaker.getX(),poseToSpeaker.getY());
        if (distanceToSpeaker < 1.0) {
            shootingAngle = 4.0*(distanceToSpeaker-0.7)-22.0;
        } 
        else if (distanceToSpeaker < 1.5) {
            shootingAngle = 4.8*(distanceToSpeaker-0.7)-22.0;
        }
        else if (distanceToSpeaker < 2.0) {
            shootingAngle = 4.4*(distanceToSpeaker-0.7)-22.0;
        }
        else if (distanceToSpeaker < 3.0) {
            shootingAngle = 3.3*(distanceToSpeaker-0.7)-22.0;
        }
        else if (distanceToSpeaker < 3.5) {
            shootingAngle = 2.8*(distanceToSpeaker-0.7)-22.0;
        }
        else {
            shootingAngle = 2.6*(distanceToSpeaker-0.7)-22.0;
        }
        /*
         * 0.8, -9
         * 2.79240671479449, -6.409871270767164 too high, 
         * 3.015658735575344, -6.119643643752052
         * 4.118724891630674, -4.685657640880123 too low
         * 4.284403100159929, -5.51559689984007 too high
         * 5.073239844268676, -4.299436171304455
         */
        SmartDashboard.putNumber("distanceToSpeaker", distanceToSpeaker);
        SmartDashboard.putNumber("Estimated Shooter Angle", shootingAngle);
        return shootingAngle;
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
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Field", m_field);
        

        for(SwerveModuleInterface mod : mSwerveMods){
            var moduleNumber = mod.getModuleNumber();
            SmartDashboard.putNumber("Mod " + moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + moduleNumber + " Absolute", Conversions.sparkToDegrees(mod.getAbsolutePosition(), Constants.SwerveConstants.angleGearRatio));
        }
    }
}
