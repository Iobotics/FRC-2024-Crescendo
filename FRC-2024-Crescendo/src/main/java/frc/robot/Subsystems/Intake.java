// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class Intake extends SubsystemBase{
    private CANSparkMax upperIntake;
    private CANSparkMax upperShoot;
    private CANSparkMax lowerIntake;
    private CANSparkMax lowerShoot;
    private CANSparkMax rightArm;
    private CANSparkMax leftArm;

    private DigitalInput optical;

    private RelativeEncoder absArmEncoder;
    private SparkPIDController rAPID;
    private SparkPIDController lAPID;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Intake(){
        upperIntake = new CANSparkMax(Constants.IntakeConstants.kUI, MotorType.kBrushless);
        upperShoot = new CANSparkMax(Constants.IntakeConstants.kUS, MotorType.kBrushless);
        lowerIntake = new CANSparkMax(Constants.IntakeConstants.kLI, MotorType.kBrushless);
        lowerShoot = new CANSparkMax(Constants.IntakeConstants.kLS, MotorType.kBrushless);
        rightArm = new CANSparkMax(Constants.IntakeConstants.RA, MotorType.kBrushless);
        leftArm = new CANSparkMax(Constants.IntakeConstants.LA, MotorType.kBrushless);
        optical = new DigitalInput(0);

        absArmEncoder = rightArm.getEncoder();

        rAPID = rightArm.getPIDController();
        configPID(rAPID);
        lAPID = leftArm.getPIDController();
        configPID(lAPID);

        upperIntake.restoreFactoryDefaults();
        upperShoot.restoreFactoryDefaults();
        lowerIntake.restoreFactoryDefaults();
        lowerShoot.restoreFactoryDefaults();
        rightArm.restoreFactoryDefaults();
        leftArm.restoreFactoryDefaults();

        upperIntake.setInverted(false);
        upperShoot.setInverted(false);
        lowerIntake.setInverted(true);
        lowerShoot.setInverted(true);
        rightArm.setInverted(false);
        leftArm.setInverted(false);

        upperIntake.setIdleMode(IdleMode.kCoast);
        upperShoot.setIdleMode(IdleMode.kCoast);
        lowerIntake.setIdleMode(IdleMode.kCoast);
        lowerShoot.setIdleMode(IdleMode.kCoast);
        rightArm.setIdleMode(IdleMode.kBrake);
        leftArm.setIdleMode(IdleMode.kBrake);

        rightArm.setOpenLoopRampRate(1);
        rightArm.setClosedLoopRampRate(0.5);
        leftArm.setOpenLoopRampRate(1);
        leftArm.setClosedLoopRampRate(0.5);

        rAPID.setFeedbackDevice(absArmEncoder);
        lAPID.setFeedbackDevice(absArmEncoder);

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

        // set PID coefficients
        rAPID.setP(kP);
        rAPID.setI(kI);
        rAPID.setD(kD);
        rAPID.setIZone(kIz);
        rAPID.setFF(kFF);
        rAPID.setOutputRange(kMinOutput, kMaxOutput);

        //smart motion values
        int smartMotionSlot = 0;
        rAPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        rAPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        rAPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        rAPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        upperIntake.burnFlash();
        upperShoot.burnFlash();
        lowerIntake.burnFlash();
        lowerShoot.burnFlash();
        rightArm.burnFlash();
        leftArm.burnFlash();
    }

    public void configPID(SparkPIDController pid){
        pid.setP(0.01);
        pid.setI(0);
        pid.setD(0);
        pid.setFF(0);

    }

    public void armSpeed(double power){
        rightArm.set(power);
        leftArm.set(power);
    }

    public void setArmPos(double pos){
        rAPID.setReference(pos, ControlType.kSmartMotion);
        lAPID.setReference(pos, ControlType.kSmartMotion);
    }

    public void setISpeed(double power, boolean enabled, boolean direction){
        if(enabled){
            do{
                lowerIntake.set(power);
                upperIntake.set(power);
            }while(optic() == direction);
            if(direction){
                lowerIntake.set(power);
                upperIntake.set(power);
                Timer.delay(0.1);
            }
            stop();
        }
        else if(!enabled){
            lowerIntake.set(power);
            upperIntake.set(power);
        }
    }

    public void pulse(double power, double repeat){
        for(int i = 0; i < repeat; i++){
            lowerIntake.set(power);
            upperIntake.set(power);
            Timer.delay(0.15);
            lowerIntake.set(0);
            upperIntake.set(0);
            Timer.delay(0.125);
            lowerIntake.set(-power/2);
            upperIntake.set(-power/2);
            Timer.delay(0.1);
            lowerIntake.set(0);
            upperIntake.set(0);
            Timer.delay(0.225);
        }
    }

    public void setSSpeed(double power){
        lowerShoot.set(power);
        upperShoot.set(power);
    }

    public void stop(){
        lowerIntake.set(0);
        upperIntake.set(0);
        lowerShoot.set(0);
        upperShoot.set(0);
    }

    public boolean optic(){
        return(optical.get());
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("Upper Intake", upperIntake.getEncoder().getPosition());
    }
    
}
