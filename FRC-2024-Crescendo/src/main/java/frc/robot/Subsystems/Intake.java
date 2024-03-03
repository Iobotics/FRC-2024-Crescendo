// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import java.util.concurrent.CompletableFuture;


/** Add your docs here. */
public class Intake extends SubsystemBase{
    private CANSparkMax upperIntake;
    private CANSparkMax lowerIntake;
    private DigitalInput optical;

    public Intake(){
        upperIntake = new CANSparkMax(Constants.IntakeConstants.kUI, MotorType.kBrushless);
        lowerIntake = new CANSparkMax(Constants.IntakeConstants.kLI, MotorType.kBrushless);
        optical = new DigitalInput(0);

        upperIntake.restoreFactoryDefaults();
        lowerIntake.restoreFactoryDefaults();

        upperIntake.setInverted(false);
        lowerIntake.setInverted(false);

        upperIntake.setIdleMode(IdleMode.kCoast);
        lowerIntake.setIdleMode(IdleMode.kCoast);

        upperIntake.burnFlash();
        lowerIntake.burnFlash();
    }

    private ExecutorService executorService = Executors.newSingleThreadExecutor();

    public void setISpeed(double power, boolean enabled, boolean direction){
        //executorService.shutdownNow();
        if(enabled){
            CompletableFuture.runAsync(() -> {
                while (optic() == direction) {
                    lowerIntake.set(power);
                    upperIntake.set(power);
                }
                stopI();
            });
        }
        else if(!enabled){
            lowerIntake.set(power);
            upperIntake.set(power);
        }
    }

    public void pulse(double power, double repeat){
        //executorService.shutdownNow();
        //executorService.execute(() -> {
        CompletableFuture.runAsync(() -> {
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
        });
        // executorService.shutdownNow();

        // for(int i = 0; i < repeat; i++){
        //     setISpeed(power, true, true);
        //     setISpeed(power, true, false);
        //     setISpeed(0, false, false);
        // }
    }

    public void stopI(){
        //executorService.shutdownNow();
        lowerIntake.set(0);
        upperIntake.set(0);
    }

    public boolean optic(){
        return(optical.get());
    }

    public void setIntakeRaw(double speed){
        lowerIntake.set(speed);
        upperIntake.set(speed);
    }

    @Override
    public void periodic(){
    }
    
}