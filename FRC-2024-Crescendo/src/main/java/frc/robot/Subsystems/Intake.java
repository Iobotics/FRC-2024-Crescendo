// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

/** Add your docs here. */
public class Intake {
    private CANSparkMax upperLeft;
    private CANSparkMax upperRight;
    private CANSparkMax lowerLeft;
    private CANSparkMax lowerRight;

    public Intake(){
        upperLeft = new CANSparkMax(Constants.IntakeConstants.kUL, MotorType.kBrushless);
        upperRight = new CANSparkMax(Constants.IntakeConstants.kUR, MotorType.kBrushless);
        lowerLeft = new CANSparkMax(Constants.IntakeConstants.kLL, MotorType.kBrushless);
        lowerRight = new CANSparkMax(Constants.IntakeConstants.kLR, MotorType.kBrushless);

        upperLeft.restoreFactoryDefaults();
        upperRight.restoreFactoryDefaults();
        lowerLeft.restoreFactoryDefaults();
        lowerRight.restoreFactoryDefaults();

        upperLeft.setInverted(false);
        upperRight.setInverted(false);
        lowerLeft.setInverted(false);
        lowerRight.setInverted(false);

        upperLeft.setIdleMode(IdleMode.kCoast);
        upperRight.setIdleMode(IdleMode.kCoast);
        lowerLeft.setIdleMode(IdleMode.kCoast);
        lowerRight.setIdleMode(IdleMode.kCoast);


        upperRight.follow(upperLeft);
        lowerRight.follow(lowerLeft);

    }

    public void setSpeed(double speed){
        upperLeft.set(speed);
        lowerLeft.set(speed);
    }

    public void stop(){
        upperLeft.stopMotor();
        lowerLeft.stopMotor();
    }

    
    
}
