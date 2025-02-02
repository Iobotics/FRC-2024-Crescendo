// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwifferConstants;

/** Add your docs here. */
public class Wrist extends SubsystemBase{
    private CANSparkMax wrist;
    public SparkPIDController wristPID;
    public double kPWrist, kIWrist, kDWrist, kIzWrist, kFFWrist, kMaxOutputWrist, kMinOutputWrist, maxVelWrist, maxAccWrist, allowedErrWrist;
    int smartMotionSlotWrist = 0;

    public Wrist(){
        wrist = new CANSparkMax(SwifferConstants.kWrist, MotorType.kBrushless);

        wrist.setIdleMode(IdleMode.kBrake);

        wrist.restoreFactoryDefaults();

        wrist.setOpenLoopRampRate(0.15);
        wrist.setClosedLoopRampRate(0);

        wristPID = wrist.getPIDController();

        wristPID.setFeedbackDevice(wrist.getEncoder());


        wrist.setSmartCurrentLimit(30);

        // ---Wrist PID and Smart Motion Setup--- //

        kPWrist = 1e-4; //proportional gain
        kIWrist = 0; //integral gain
        kDWrist = 4e-5; //derivative gain
        kIzWrist = 0.0; //don't touch this
        kFFWrist = 0.000156; //don't touch this either
        kMaxOutputWrist = 1; //maximum power
        kMinOutputWrist = -1; //minimum power
        maxVelWrist = 10000; //rpm
        maxAccWrist = 6000; //rpm per second
        allowedErrWrist = 0.01; //revolutions

        wristPID.setP(kPWrist);
        wristPID.setI(kIWrist);
        wristPID.setD(kDWrist);
        wristPID.setFF(kFFWrist);
        wristPID.setOutputRange(kMinOutputWrist, kMaxOutputWrist);

        wristPID.setSmartMotionMaxVelocity(maxVelWrist, 0); //tune the max cruise velocity (RPM)
        wristPID.setSmartMotionMaxAccel(maxAccWrist, 0); //tune the max smart motion acceleration (RPM per second)
        wristPID.setSmartMotionAllowedClosedLoopError(allowedErrWrist, 0); //set the closed loop error

        wrist.burnFlash(); //flash the new parameters to the sparkmax
    }

    // ---Wrist Functions--- //

    //Raw speed function
    public void setPowerWrist(double speed){
        wrist.set(speed);
    }

    //Stop function
    public void stopWrist(){
        wrist.set(0);  
    }

    public double getWristPos(){
        return (wrist.getEncoder().getPosition());
    }
    
    //Preset position function
    public void presetWrist(double rev){
        wristPID.setReference(rev, ControlType.kSmartMotion);
    }

    //Error check function
    public boolean isWristWithinError(double target, double error){
        return (Math.abs(target - getWristPos()) <= error);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Wrist Pos", getWristPos());  //Update wrist position
    }

}
