// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ---Setup and functions for the Swiffer arm, wrist, and rollers--- //

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkAbsoluteEncoder.Type;
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

  private AbsoluteEncoder armEncoder;
  private AbsoluteEncoder wristEncoder;
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
    wrist.setIdleMode(IdleMode.kCoast);
    roller.setIdleMode(IdleMode.kBrake);

    arm.restoreFactoryDefaults();
    wrist.restoreFactoryDefaults();
    roller.restoreFactoryDefaults();

    // Set the ramp rate (time from 0 to full throttle)
    arm.setOpenLoopRampRate(1);
    arm.setClosedLoopRampRate(0);
    wrist.setOpenLoopRampRate(0.5);
    wrist.setClosedLoopRampRate(1);

    //Set up bottom limit switch for the arm
    armBottomLimit = arm.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    arm.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    arm.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    armBottomLimit.enableLimitSwitch(false);

    //Set up PID controllers for the arm and wrist
    armPID = arm.getPIDController();
    wristPID = wrist.getPIDController();

    //Set up encoders for the arm and wrist (through bore encoder)
    armEncoder = arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    wristEncoder = wrist.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    armPID.setFeedbackDevice(armEncoder);
    wristPID.setFeedbackDevice(wristEncoder);

    //Set up device current limits
    arm.setSmartCurrentLimit(30);
    wrist.setSmartCurrentLimit(30);
    roller.setSmartCurrentLimit(20);

    // // ---Arm PID and Smart Motion Setup--- //
    
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

    kPWrist = 0.000001; //proportional gain
    kIWrist = 0.0; //integral gain
    kDWrist = 0.0; //derivative gain
    kIzWrist = 0.0; //don't touch this
    kFFWrist = 0.0; //don't touch this either
    kMaxOutputWrist = 1; //maximum power
    kMinOutputWrist = -1; //minimum power
    maxVelWrist = 5000; //rpm
    maxAccWrist = 500; //rpm per second
    allowedErrWrist = 0.1; //revolutions

    wristPID.setP(kPWrist);
    wristPID.setI(kIWrist);
    wristPID.setD(kDWrist);
    wristPID.setFF(kFFWrist);
    wristPID.setOutputRange(kMinOutputWrist, kMaxOutputWrist);

    wristPID.setSmartMotionMaxVelocity(2500, 0); //tune the max cruise velocity (RPM)
    wristPID.setSmartMotionMaxAccel(2500, 0); //tune the max smart motion acceleration (RPM per second)
    wristPID.setSmartMotionAllowedClosedLoopError(allowedErrWrist, 0); //set the closed loop error

    wrist.burnFlash(); //flash the new parameters to the sparkmax
  }

  // ---Arm Functions--- //

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
    return arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed(); //FIX THIS
  }

  //Error check function
  public boolean isArmWithinError(double target, double error){
      return Math.abs(target = armEncoder.getPosition()) <= error;
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
    return wristEncoder.getPosition();
  }
  
  //Preset position function
  public void presetWrist(double rev){
    wristPID.setReference(rev, CANSparkMax.ControlType.kSmartMotion, 0, 0);
  }

  //Error check function
  public boolean isWristWithinError(double target, double error){
    return Math.abs(target - wristEncoder.getPosition()) <= error;
  }

  // ---Roller Functions---//

  //Set roller power function
  public void setPowerRoller(double speed){
    //do{
      roller.set(speed);
    //}while(isNoteDetected() == true);
    
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
