// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        gyro.configFactoryDefault();
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

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
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

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getRoll(){
        return gyro.getRoll();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void stop(){
        for(SwerveModule mod : mSwerveMods){
            mod.stop();
        }
    }

    public boolean isTargetAchieved(double distance, double error) {
        // SmartDashboard.putNumber("Second:",distance);
        return(Math.abs(distance - mSwerveMods[0].getPosition().distanceMeters) <= error);
    }

    // boolean that checks if the robot is engaged on the platform
    public boolean isEngaged(double roll , double error){
        return(Math.abs(roll - getRoll()) <= error);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        SmartDashboard.putNumber("Roll", getRoll());
        
        /*for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }*/

        // SmartDashboard.putNumber("distance", mSwerveMods[0].getPosition().distanceMeters);

        // SmartDashboard.putNumber("Front Left Cancoder", mSwerveMods[0].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Front Left Integrated", mSwerveMods[0].getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Front Left Velocity", mSwerveMods[0].getState().speedMetersPerSecond);

        // SmartDashboard.putNumber("Front Right Cancoder", mSwerveMods[1].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Front Right Integrated", mSwerveMods[1].getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Front Right Velocity", mSwerveMods[1].getState().speedMetersPerSecond);

        // SmartDashboard.putNumber("Back Left Cancoder", mSwerveMods[2].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Back Left Integrated", mSwerveMods[2].getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Back Left Velocity", mSwerveMods[2].getState().speedMetersPerSecond);

        // SmartDashboard.putNumber("Back Right Cancoder", mSwerveMods[3].getCanCoder().getDegrees());
        // SmartDashboard.putNumber("Back Right Integrated", mSwerveMods[3].getPosition().angle.getDegrees());
        // SmartDashboard.putNumber("Back Right Velocity", mSwerveMods[3].getState().speedMetersPerSecond);

    }
}
