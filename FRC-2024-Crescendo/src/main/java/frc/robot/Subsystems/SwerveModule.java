// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utils.CANCoderUtil;
import frc.robot.Utils.CANSparkMaxUtil;
import frc.robot.Utils.ModuleState;
import frc.robot.Utils.SwerveModuleConstants;
import frc.robot.Utils.CANCoderUtil.CCUsage;
import frc.robot.Utils.CANSparkMaxUtil.Usage;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;

    private SparkMaxPIDController driveController;
    private SparkMaxPIDController angleController;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, "Carnivore");
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        driveController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = ModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            mDriveMotor.set(percentOutput);    
        }
        else {
            driveController.setReference(
            desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
        angleController.setP(Constants.SwerveConstants.angleKP);
        angleController.setI(Constants.SwerveConstants.angleKI);
        angleController.setD(Constants.SwerveConstants.angleKD);
        angleController.setFF(Constants.SwerveConstants.angleKF);
        mAngleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        

        mDriveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mDriveMotor, Usage.kAll);
        mDriveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
        driveController.setP(Constants.SwerveConstants.angleKP);
        driveController.setI(Constants.SwerveConstants.angleKI);
        driveController.setD(Constants.SwerveConstants.angleKD);
        driveController.setFF(Constants.SwerveConstants.angleKF);
        mDriveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    //everything below here is fine.

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            driveEncoder.getVelocity(), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            getAngle()
        );
    }

    public void stop(){
        mDriveMotor.stopMotor();
    }
}