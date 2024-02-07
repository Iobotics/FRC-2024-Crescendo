// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ---Setup and functions for the Swiffer arm, wrist, and rollers--- //

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwifferConstants;

public class Swiffer extends SubsystemBase {

  private CANSparkMax arm;
  private CANSparkMax wrist;
  private CANSparkMax roller;

  public SparkPIDController armPID;
  public SparkPIDController wristPID;

  private RelativeEncoder armEncoder;
  private RelativeEncoder wristEncoder;
  private SparkLimitSwitch armBottomLimit;

  public double kPArm, kIArm, kDArm, kIzArm, kFFArm, kMaxOutputArm, kMinOutputArm, maxVelArm, maxAccArm, allowedErrArm;
  int smartMotionSlotArm = 1;
  public double kPWrist, kIWrist, kDWrist, kIzWrist, kFFWrist, kMaxOutputWrist, kMinOutputWrist, maxVelWrist, maxAccWrist, allowedErrWrist;
  int smartMotionSlotWrist = 0;
   
  private DigitalInput swifferIntake;
 
  /** Creates a new Swiffer. */
  public Swiffer() {

    //Declare channel for optical limit switch in the swiffer
    swifferIntake = new DigitalInput(SwifferConstants.kDigitalInput);

    //Initialize arm, wrist, and roller
    arm = new CANSparkMax(SwifferConstants.kArm, MotorType.kBrushless);
    wrist = new CANSparkMax(SwifferConstants.kWrist, MotorType.kBrushless);
    roller = new CANSparkMax(SwifferConstants.kRoller, MotorType.kBrushless);
    
    arm.setIdleMode(IdleMode.kBrake);
    wrist.setIdleMode(IdleMode.kBrake);
    roller.setIdleMode(IdleMode.kBrake);

    arm.restoreFactoryDefaults();
    wrist.restoreFactoryDefaults();
    roller.restoreFactoryDefaults();

    //Set the ramp rate (time from 0 to full throttle)
    arm.setOpenLoopRampRate(1);
    arm.setClosedLoopRampRate(1);
    wrist.setOpenLoopRampRate(0.75);
    wrist.setClosedLoopRampRate(0.75);

    //Set up bottom limit switch for the arm
    armBottomLimit = arm.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    arm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armBottomLimit.enableLimitSwitch(false);

    //Set up PID controllers for the arm and wrist
    armPID = arm.getPIDController();
    wristPID = wrist.getPIDController();

    //Set up encoders for the arm and wrist (through bore encoder)
    armEncoder = arm.getAlternateEncoder(2048);
    wristEncoder = wrist.getEncoder();
    armPID.setFeedbackDevice(armEncoder);
    wristPID.setFeedbackDevice(wristEncoder);

    //Set up device current limits
    arm.setSmartCurrentLimit(20);
    wrist.setSmartCurrentLimit(20);
    roller.setSmartCurrentLimit(10);

    // ---Arm PID and Smart Motion Setup--- //
    
    kPArm = 0.000001; //proportional gain
    kIArm = 0.0; //integral gain
    kDArm = 0.0; //derivative gain
    kIzArm = 0.0; //don't touch this
    kFFArm = 0.0; //don't touch this either
    kMaxOutputArm = 1; //maximum power
    kMinOutputArm = -1; //minimum power
    maxVelArm = 1000; //rpm
    maxAccArm = 500; //rpm per second
    allowedErrArm = 0.1; //inches

    armPID.setP(kDArm);
    armPID.setI(kIArm);
    armPID.setD(kDArm);
    armPID.setOutputRange(kMinOutputArm, kMaxOutputArm);

    armPID.setSmartMotionMaxVelocity(1000, smartMotionSlotArm); //tune the max cruise velocity (RPM)
    armPID.setSmartMotionMinOutputVelocity(0, smartMotionSlotArm); //minimum output velocity is 0... duh
    armPID.setSmartMotionMaxAccel(1000, smartMotionSlotArm); //tune the max smart motion acceleration (RPM per second)
    armPID.setSmartMotionAllowedClosedLoopError(allowedErrArm, smartMotionSlotArm); //set the closed loop error

    arm.burnFlash(); //flash the new parameters to the sparkmax

    // ---Wrist PID and Smart Motion Setup--- //

    kPWrist = 0.010; //proportional gain
    kIWrist = 0.00000001; //integral gain
    kDWrist = 0.0; //derivative gain
    kIzWrist = 0.0; //don't touch this
    kFFWrist = 0.4; //don't touch this either
    kMaxOutputWrist = 1; //maximum power
    kMinOutputWrist = -1; //minimum power
    maxVelWrist = 5000; //rpm
    maxAccWrist = 500; //rpm per second
    allowedErrWrist = 0.1; //inches

    wristPID.setP(kPWrist);
    wristPID.setI(kIWrist);
    wristPID.setD(kDWrist);
    wristPID.setOutputRange(kMinOutputWrist, kMaxOutputWrist);

    wristPID.setSmartMotionMaxVelocity(maxVelWrist, smartMotionSlotWrist); //tune the max cruise velocity (RPM)
    wristPID.setSmartMotionMinOutputVelocity(0, smartMotionSlotWrist); //minimum output velocity is 0... duh
    wristPID.setSmartMotionMaxAccel(maxAccWrist, smartMotionSlotWrist); //tune the max smart motion acceleration (RPM per second)
    wristPID.setSmartMotionAllowedClosedLoopError(allowedErrWrist, smartMotionSlotWrist); //set the closed loop error

    wrist.burnFlash(); //flash the new parameters to the sparkmax
  }

  // ---Arm Functions--- //

  //Preset position function
  public void presetArm(double targetPosition){
    armPID.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion, smartMotionSlotArm, kFFArm);
  }

  //Get arm position function
  public double getArmPos(){
    return armEncoder.getPosition();
  }

  //Tapering speed function
  public void speedArm(){

  }

  //Raw speed function
  public void setPowerArm(){

  }

  //Bottom limit switch check function
  public boolean isArmLowerLimit(){
    return arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed(); //FIX THIS
  }

  //Error check function
  public boolean isArmWithinError(double targetInch, double error){
      return Math.abs(armEncoder.getPosition()) <= error;
    }

  //Stop function
  public void stopArm(){
    arm.set(0);
  }

  // ---Wrist Functions--- //

  //Preset position function
  public void presetWrist(double targetPosition){
    wristPID.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion, smartMotionSlotWrist, kFFWrist);
  }

  public double getWristPos(){
    return wristEncoder.getPosition();
  }
  
  //Raw speed function
  public void setPowerWrist(double speed){
    wrist.set(speed);
  }

  //Error check function
  public boolean isWristWithinError(double targetInch, double error){
    return Math.abs(wristEncoder.getPosition()) <= error;
  }

  //Stop function
  public void stopWrist(){
    wrist.set(0);  
  }

  // ---Roller Functions---//

  //Set roller power function
  public void setPowerRoller(double speed){
    roller.set(speed);
  }

  //Note detection function
  public boolean isNoteDetected(){
    return (swifferIntake.get());
  }

  //Stop roller function
  public void stopRoller(){
    roller.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmPos", getArmPos()); //Update arm position
    SmartDashboard.putNumber("WristPos", getWristPos());  //Update wrist position
    SmartDashboard.putBoolean("Note?", isNoteDetected()); //Update note intake status (FIX THIS)
  }
}
