// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
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

    //sets up arm sparkmax
    public Extension(){
        arm = new CANSparkMax(SwifferConstants.kArm, MotorType.kBrushless); //set up sparkmax

        //default when restarted or rebooted
        arm.restoreFactoryDefaults(); 

        arm.setInverted(false); //direction

        arm.setIdleMode(IdleMode.kBrake); //idle mode setting
         
        arm.setOpenLoopRampRate(0.15); //ramprate controls
        arm.setClosedLoopRampRate(0);

        //Set up bottom limit switch for the arm
        armBottomLimit = arm.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed); 
        armBottomLimit.enableLimitSwitch(true);

        armPID = arm.getPIDController(); //use PID

        armEncoder = arm.getEncoder(); //assign encoder
        armPID.setFeedbackDevice(armEncoder); //recieve encoder data

        arm.setSmartCurrentLimit(30); //limit or reduce power
 
        //setup PID
        kPArm = 2e-4; //proportional gain
        kIArm = 0; //integral gain
        kDArm = 4e-6; //derivative gain
        kIzArm = 0.0; //don't touch this
        kFFArm = 0.000156; //don't touch this either
        kMaxOutputArm = 1; //maximum power
        kMinOutputArm = -1; //minimum power
        maxVelArm = 10000; //rpm
        maxAccArm = 15000; //rpm per second
        allowedErrArm = 0.1; //inches

        //use pid for arm
        armPID.setP(kPArm);
        armPID.setI(kIArm);
        armPID.setD(kDArm);
        armPID.setOutputRange(kMinOutputArm, kMaxOutputArm);

        armPID.setSmartMotionMaxVelocity(maxVelArm, smartMotionSlotArm); //tune the max cruise velocity (RPM)
        armPID.setSmartMotionMinOutputVelocity(0, smartMotionSlotArm); //minimum output velocity is 0... duh
        armPID.setSmartMotionMaxAccel(maxAccArm, smartMotionSlotArm); //tune the max smart motion acceleration (RPM per second)
        armPID.setSmartMotionAllowedClosedLoopError(allowedErrArm, smartMotionSlotArm); //set the closed loop error

        //burn into sparkmax
        arm.burnFlash(); //flash the new parameters to the sparkmax
    }

    //Raw speed function
    public void setPowerArm(double speed){
        arm.set(-speed);
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
        armPID.setReference(targetPosition, ControlType.kSmartMotion, smartMotionSlotArm, kFFArm);
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
        SmartDashboard.putNumber("Ext Pos", arm.getEncoder().getPosition()); //Update arm position
    }
}
