// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax climber1; //left
  private CANSparkMax climber2; //right

  private RelativeEncoder climber1E;
  private RelativeEncoder climber2E;
  private SparkPIDController climb1P;
  private SparkPIDController climb2P;

  private SparkLimitSwitch climber1LS;
	private SparkLimitSwitch climber2LS;
	private Servo servo1;
	private Servo servo2;

  /** Creates a new Climber. */
  public Climber() {
    //create new sparkmax instances
    climber1 = new CANSparkMax(ClimberConstants.kClimber1, MotorType.kBrushless);
    climber1.restoreFactoryDefaults();

    climber2 = new CANSparkMax(ClimberConstants.kClimber2, MotorType.kBrushless);
    climber2.restoreFactoryDefaults();

    //Creates servo instances
    servo1 = new Servo(0);
    servo2 = new Servo(1);

    //Reset sparkmaxes
    climber1.restoreFactoryDefaults();
    climber2.restoreFactoryDefaults();

    //Direction
    climber1.setInverted(false);
    climber2.setInverted(true);

    //Idlemode: Can be Brake or Coast
    climber1.setIdleMode(IdleMode.kBrake);
    climber2.setIdleMode(IdleMode.kBrake);

    //Set RampRate:
    //Open: Manual Control
    //Closed: Closed loop control i.e. PID
    climber1.setClosedLoopRampRate(0);
    climber1.setOpenLoopRampRate(1);
    climber2.setClosedLoopRampRate(0);
    climber2.setOpenLoopRampRate(1);

    //reads encoder 
    climber1E = climber1.getEncoder(); 
    climber2E = climber2.getEncoder();

    //assign PID Controller
    climb1P = climber1.getPIDController();
    PIDConfiguration(climb1P);
    climb2P = climber2.getPIDController();
    PIDConfiguration(climb2P);

   //sends encoder readings to PID controller
    climb1P.setFeedbackDevice(climber1E);
    climb2P.setFeedbackDevice(climber2E);

   //assigns limit switches to climbers
    climber1LS = climber1.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    climber2LS = climber2.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
   
    //enables limit switch
    climber1LS.enableLimitSwitch(true);
    climber2LS.enableLimitSwitch(true);

    //Burns all the values above onto the sparkmax
    climber1.burnFlash();
    climber2.burnFlash();

    
  }

   //sets up PID Controller
  public void PIDConfiguration(SparkPIDController pid){

    pid.setP(1e-7);
    pid.setI(0);
    pid.setD(0);
    pid.setIZone(0);
    pid.setFF(0);
    pid.setOutputRange(1, -1);

    pid.setSmartMotionMaxVelocity(2000, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(2000, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.1, 0);

  }

   //set up climber position function
  public void climbPOS(double posL, double posR){
  climb1P.setReference(posL, ControlType.kSmartMotion);
  climb2P.setReference(posR, ControlType.kSmartMotion);
  }
   
   //recieve position value 
  public double climbValue(){
    return(climber1E.getPosition()*Constants.ClimberConstants.kCGearRatio);
  }

  //checks if the climber is at the right position or how much further it needs to go
  public boolean isClimbWithinError(double target, double error){
    return (Math.abs(target - climbValue()) <= error);
  }

  //set power to each climber
  public void setPower(double speed){
    climber1.set(speed);
    climber2.set(speed);
  }

  //stops climber
  public void stopClimber(){
    climber1.set(0);
    climber2.set(0);
  }
   
  //left climber speed
  public void setPowerL(double speed){
   climber1.set(-speed);
  }

  //stops left climber
  public void stopL(){
    climber1.set(0);
  }

  //right climber speed  
  public void setPowerR(double speed){
    climber2.set(speed);
  }
  
  //stop right climber
  public void stopR(){
    climber2.set(0);
  }

  //set servo positions
  public void zeroRatchets() {
    servo1.set(1500);
    servo2.set(1500);
  }
  
  //lock servo
  public void lock(){
    servo1.setPulseTimeMicroseconds(1625);
    servo2.setPulseTimeMicroseconds(1375);
  }

  //unlock servo
  public void unlock(){
    servo1.setPulseTimeMicroseconds(1500); //L
    servo2.setPulseTimeMicroseconds(1500); //R
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LClimb", climber1E.getPosition());
    SmartDashboard.putNumber("RClimb", climber2E.getPosition());
  }
}