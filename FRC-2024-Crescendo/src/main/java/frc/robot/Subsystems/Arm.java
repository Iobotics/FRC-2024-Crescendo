// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class Arm extends SubsystemBase{
    private CANSparkMax rightArm;
    private CANSparkMax leftArm;
    private SparkPIDController rAPID;
    private SparkPIDController lAPID;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    private AbsoluteEncoder absEncoder;

    public Arm(){
        rightArm = new CANSparkMax(Constants.IntakeConstants.kRA, MotorType.kBrushless);
        leftArm = new CANSparkMax(Constants.IntakeConstants.kLA, MotorType.kBrushless);

        //absEncoder = leftArm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        rightArm.restoreFactoryDefaults();
        leftArm.restoreFactoryDefaults();

        rightArm.setInverted(false);
        leftArm.setInverted(true);

        rightArm.setIdleMode(IdleMode.kBrake);
        leftArm.setIdleMode(IdleMode.kBrake);

        rightArm.setOpenLoopRampRate(0.5);
        rightArm.setClosedLoopRampRate(0);
        leftArm.setOpenLoopRampRate(0.5);
        leftArm.setClosedLoopRampRate(0);

        // PID coefficients
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;

        rAPID = rightArm.getPIDController();
        configPID(rAPID);
        lAPID = leftArm.getPIDController();
        configPID(lAPID);

        lAPID.setFeedbackDevice(leftArm.getEncoder());

        rightArm.burnFlash();
        leftArm.burnFlash();
    }

    public void configPID(SparkPIDController pid){
        // set PID coefficients
        pid.setP(kP);
        pid.setI(kI);
        pid.setD(kD);
        pid.setIZone(kIz);
        pid.setFF(kFF);
        pid.setOutputRange(kMinOutput, kMaxOutput);

        //smart motion values
        int smartMotionSlot = 0;
        pid.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pid.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pid.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pid.setSmartMotionAllowedClosedLoopError(0.01, smartMotionSlot);

    }

    public void armSpeed(double power){
        rightArm.set(power);
        leftArm.set(power);

        // rAPID.setReference(power, ControlType.kDutyCycle);
        // rAPID.setReference(power, ControlType.kDutyCycle);
    }

    public void setArmPos(double pos){
        rAPID.setReference(pos, ControlType.kSmartMotion);
        lAPID.setReference(pos, ControlType.kSmartMotion);
    }

    public double getArmPos(){
        return(absEncoder.getPosition() * IntakeConstants.kArmGearRatio);
    }

    public void stopA(){
        rightArm.set(0);
        leftArm.set(0);
    }

    public boolean isArmWithinError(double target, double error){
        return (Math.abs(target - leftArm.getEncoder().getPosition()) <= error);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Pos", leftArm.getEncoder().getPosition());
    }
}
