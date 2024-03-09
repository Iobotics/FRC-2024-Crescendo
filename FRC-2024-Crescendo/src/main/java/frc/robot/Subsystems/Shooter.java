// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.Character.Subset;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Shooter extends SubsystemBase{
    private CANSparkMax upperShoot;
    private CANSparkMax lowerShoot;

    public Shooter(){
        upperShoot = new CANSparkMax(Constants.IntakeConstants.kUS, MotorType.kBrushless);
        lowerShoot = new CANSparkMax(Constants.IntakeConstants.kLS, MotorType.kBrushless);

        upperShoot.restoreFactoryDefaults();
        lowerShoot.restoreFactoryDefaults();

        upperShoot.setInverted(false);
        lowerShoot.setInverted(false);

        upperShoot.setIdleMode(IdleMode.kCoast);
        lowerShoot.setIdleMode(IdleMode.kCoast);

        upperShoot.burnFlash();
        lowerShoot.burnFlash();
    }

    public void setSSpeed(double power){
        lowerShoot.set(power);
        upperShoot.set(power);
    }

    public void stopS(){
        lowerShoot.set(0);
        upperShoot.set(0);
    }

}
