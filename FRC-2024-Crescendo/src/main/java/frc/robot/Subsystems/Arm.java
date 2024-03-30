// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

/** Add your docs here. */
public class Arm extends SubsystemBase{
    private CANSparkMax rightArm;
    private CANSparkMax leftArm;
    private SparkPIDController rAPID;
    private SparkPIDController lAPID;


    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Arm(){
        //Create new sparkmax instances
        rightArm = new CANSparkMax(Constants.IntakeConstants.kRA, MotorType.kBrushless);
        leftArm = new CANSparkMax(Constants.IntakeConstants.kLA, MotorType.kBrushless);

        //Reset sparkmaxes
        rightArm.restoreFactoryDefaults();
        leftArm.restoreFactoryDefaults();

        rightArm.setSmartCurrentLimit(45);
        leftArm.setSmartCurrentLimit(45);

        //Direction
        rightArm.setInverted(false);
        leftArm.setInverted(false);

        //Idlemode: Can be Brake or Coast
        rightArm.setIdleMode(IdleMode.kBrake);
        leftArm.setIdleMode(IdleMode.kBrake);

        //Set RampRate:
        //Open: Manual Control
        //Closed: Closed loop control i.e. PID
        rightArm.setOpenLoopRampRate(0.5);
        rightArm.setClosedLoopRampRate(0);
        leftArm.setOpenLoopRampRate(0.5);
        leftArm.setClosedLoopRampRate(0);

        // PID coefficients
        kP = 4e-2; 
        kI = 4e-5;
        kD = 4e-5; 
        kIz = 0; 
        kFF = 0.05; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 1900; // rpm
        maxAcc = 1400;

        //configure each PID object to it's correct controller
        rAPID = rightArm.getPIDController();
        configPID(rAPID);
        lAPID = leftArm.getPIDController();
        configPID(lAPID);

        //set the motor's Hall encoder to be the feedback
        //You only need one because the two motors will turn the same amount of rotations;
        lAPID.setFeedbackDevice(leftArm.getEncoder());

        //Burns all the values above onto the sparkmax
        rightArm.burnFlash();
        leftArm.burnFlash();
    }

    //configures the PID object
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

    //Manual control: goes from 0 to 1
    public void armSpeed(double power){
        rightArm.set(power);
        leftArm.set(power);
    }

    //Closed loop control
    public void setArmPos(double pos){
        rAPID.setReference(pos, ControlType.kPosition);
        lAPID.setReference(pos, ControlType.kPosition);
    }

    public void setArmPos(Supplier<Double> posSupplier){
        setArmPos(posSupplier.get());
    }

    //returns encoder value
    public double getArmPos(){
        return(rightArm.getEncoder().getPosition());
    }

    public void stopA(){
        rightArm.set(0);
        leftArm.set(0);
    }

    //checks if the arm is at the right position. Used for closed loop control
    public boolean isArmWithinError(double target, double error){
        return (Math.abs(target - getArmPos()) <= error);
    }

    //puts the motors into brake mode
    public void brake(){
        rightArm.setIdleMode(IdleMode.kBrake);
        leftArm.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic(){
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Arm Pos", getArmPos());
    }
}
