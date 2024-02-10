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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private RelativeEncoder rArmEncoder;
    private RelativeEncoder lArmEncoder;
    private SparkPIDController rAPID;
    private SparkPIDController lAPID;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Intake(){
        upperIntake = new CANSparkMax(Constants.IntakeConstants.kUI, MotorType.kBrushless);
        upperShoot = new CANSparkMax(Constants.IntakeConstants.kUS, MotorType.kBrushless);
        lowerIntake = new CANSparkMax(Constants.IntakeConstants.kLI, MotorType.kBrushless);
        lowerShoot = new CANSparkMax(Constants.IntakeConstants.kLS, MotorType.kBrushless);
        rightArm = new CANSparkMax(Constants.IntakeConstants.kRA, MotorType.kBrushless);
        leftArm = new CANSparkMax(Constants.IntakeConstants.kLA, MotorType.kBrushless);
        optical = new DigitalInput(0);

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
        leftArm.setInverted(true);

        upperIntake.setIdleMode(IdleMode.kCoast);
        upperShoot.setIdleMode(IdleMode.kCoast);
        lowerIntake.setIdleMode(IdleMode.kCoast);
        lowerShoot.setIdleMode(IdleMode.kCoast);
        rightArm.setIdleMode(IdleMode.kBrake);
        leftArm.setIdleMode(IdleMode.kBrake);

        rightArm.setOpenLoopRampRate(0.5);
        rightArm.setClosedLoopRampRate(0);
        leftArm.setOpenLoopRampRate(0.5);
        leftArm.setClosedLoopRampRate(0);

        rArmEncoder = rightArm.getEncoder();
        lArmEncoder = leftArm.getEncoder();

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

        rAPID.setFeedbackDevice(rArmEncoder);
        lAPID.setFeedbackDevice(lArmEncoder);

        upperIntake.burnFlash();
        upperShoot.burnFlash();
        lowerIntake.burnFlash();
        lowerShoot.burnFlash();
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
        pid.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

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
            stopI();
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

    public void stopI(){
        lowerIntake.set(0);
        upperIntake.set(0);
    }
        

    public void stopS(){
        lowerShoot.set(0);
        upperShoot.set(0);
    }

    public void stopA(){
        rightArm.set(0);
        leftArm.set(0);
    }

    public boolean optic(){
        return(optical.get());
    }

    public double getArmPos(){
        return(rArmEncoder.getPosition() * IntakeConstants.kArmGearRatio);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Pos", rightArm.getEncoder().getPosition());
    }
    
}
