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
    climber1 = new CANSparkMax(ClimberConstants.kClimber1, MotorType.kBrushless);
    climber1.restoreFactoryDefaults();

    climber2 = new CANSparkMax(ClimberConstants.kClimber2, MotorType.kBrushless);
    climber2.restoreFactoryDefaults();

    servo1 = new Servo(2);
    servo2 = new Servo(1);

    climber1.setInverted(false);
    climber2.setInverted(false);

    climber1.setIdleMode(IdleMode.kBrake);
    climber2.setIdleMode(IdleMode.kBrake);

    climber1.setClosedLoopRampRate(0);
    climber1.setOpenLoopRampRate(.5);
    climber2.setClosedLoopRampRate(0);
    climber2.setOpenLoopRampRate(.5);

    climber1E = climber1.getEncoder();
    climber2E = climber2.getEncoder();

    climb1P = climber1.getPIDController();
    PIDConfiguration(climb1P);
    climb2P = climber2.getPIDController();
    PIDConfiguration(climb2P);

    climb1P.setFeedbackDevice(climber1E);
    climb2P.setFeedbackDevice(climber2E);

    climber1.burnFlash();
    climber2.burnFlash();

    
  }

  public void PIDConfiguration(SparkPIDController pid){

    pid.setP(5e-5);
    pid.setI(1e-6);
    pid.setD(0);
    pid.setIZone(0);
    pid.setFF(0);
    pid.setOutputRange(1, -1);

    pid.setSmartMotionMaxVelocity(2000, 0);
    pid.setSmartMotionMinOutputVelocity(0, 0);
    pid.setSmartMotionMaxAccel(2000, 0);
    pid.setSmartMotionAllowedClosedLoopError(0.1, 0);

  }

  public void climbPOS(double pos){
  climb1P.setReference(pos, ControlType.kSmartMotion);
  climb2P.setReference(pos, ControlType.kSmartMotion);

  }

  public double climbValue(){
    return(climber1E.getPosition()*Constants.ClimberConstants.kCGearRatio);

  }



  public void setPower(double speed){
    climber1.set(-speed);
    climber2.set(speed);
  }

  public void stop(){
    climber1.set(0);
    climber2.set(0);
  }

  public void lock(){
    servo1.setAngle(120);
    servo2.setAngle(60);
  }

  public void unlock(){
    servo1.setAngle(0); //R
    servo2.setAngle(90); //L
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Climb Position", climber1E.getPosition());
  }
}