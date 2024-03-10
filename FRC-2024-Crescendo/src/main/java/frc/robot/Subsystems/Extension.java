// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.SwifferConstants;

/** Add your docs here. */
public class Extension extends SubsystemBase{
    private CANSparkMax arm;
    public SparkPIDController armPID;
    private RelativeEncoder armEncoder;
    private SparkLimitSwitch armBottomLimit;
    public double kPArm, kIArm, kDArm, kIzArm, kFFArm, kMaxOutputArm, kMinOutputArm, maxVelArm, maxAccArm, allowedErrArm;
    int smartMotionSlotArm = 0;

    public Extension(){
        arm = new CANSparkMax(SwifferConstants.kArm, MotorType.kBrushless);

        arm.setIdleMode(IdleMode.kBrake);
        
        arm.restoreFactoryDefaults();

        arm.setOpenLoopRampRate(1);
        arm.setClosedLoopRampRate(0);

        //Set up bottom limit switch for the arm
        // armBottomLimit = arm.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        // arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        // arm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        // armBottomLimit.enableLimitSwitch(false);

        armPID = arm.getPIDController();

        armEncoder = arm.getEncoder();
        armPID.setFeedbackDevice(armEncoder);

        arm.setSmartCurrentLimit(30);

        kPArm = 5e-5; //proportional gain
        kIArm = 1e-6; //integral gain
        kDArm = 0.0; //derivative gain
        kIzArm = 0.0; //don't touch this
        kFFArm = 0.000156; //don't touch this either
        kMaxOutputArm = 1; //maximum power
        kMinOutputArm = -1; //minimum power
        maxVelArm = 1000; //rpm
        maxAccArm = 500; //rpm per second
        allowedErrArm = 0.1; //inches

        armPID.setP(kPArm);
        armPID.setI(kIArm);
        armPID.setD(kDArm);
        armPID.setOutputRange(kMinOutputArm, kMaxOutputArm);

        armPID.setSmartMotionMaxVelocity(1000, smartMotionSlotArm); //tune the max cruise velocity (RPM)
        armPID.setSmartMotionMinOutputVelocity(0, smartMotionSlotArm); //minimum output velocity is 0... duh
        armPID.setSmartMotionMaxAccel(1000, smartMotionSlotArm); //tune the max smart motion acceleration (RPM per second)
        armPID.setSmartMotionAllowedClosedLoopError(allowedErrArm, smartMotionSlotArm); //set the closed loop error

        arm.burnFlash(); //flash the new parameters to the sparkmax
    }

    //Raw speed function
    public void setPowerArm(double speed){
        arm.set(speed);
    }

    //Stop function
    public void stopArm(){
        arm.set(0);
    }

    //Get arm position function
    public double getArmPos(){
        return armEncoder.getPosition();
    } 

    //Preset position function
    public void presetArm(double targetPosition){
        armPID.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion, smartMotionSlotArm, kFFArm);
    }

    //Bottom limit switch check function
    public boolean isArmLowerLimit(){
        return arm.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed(); //FIX THIS
    }

    //Error check function
    public boolean isArmWithinError(double target, double error){
        return(Math.abs(target - arm.getEncoder().getPosition()) <= error);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ArmPos", arm.getEncoder().getPosition()); //Update arm position
    }

}
