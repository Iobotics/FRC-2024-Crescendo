// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.concurrent.CompletableFuture;


/** Add your docs here. */
public class Intake extends SubsystemBase{
    private CANSparkMax upperIntake;
    private CANSparkMax lowerIntake;
    private DigitalInput optical1;

    //setup intake
    public Intake(){

        //setup sparkmax for intake
        upperIntake = new CANSparkMax(Constants.IntakeConstants.kUI, MotorType.kBrushless);
        lowerIntake = new CANSparkMax(Constants.IntakeConstants.kLI, MotorType.kBrushless);
        optical1 = new DigitalInput(0);

        //reset to default when rebooted
        upperIntake.restoreFactoryDefaults();
        lowerIntake.restoreFactoryDefaults();

        //directions
        upperIntake.setInverted(true);
        lowerIntake.setInverted(true);

        //idle mode
        upperIntake.setIdleMode(IdleMode.kCoast);
        lowerIntake.setIdleMode(IdleMode.kCoast);

        //burn into sparkmax
        upperIntake.burnFlash();
        lowerIntake.burnFlash();
    }

    public void setISpeed(double power, boolean enabled, boolean direction){
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
        CompletableFuture.runAsync(() -> {
            for(int i = 0; i < repeat; i++){
                lowerIntake.set(power);
                upperIntake.set(power);
                Timer.delay(0.15);
                lowerIntake.set(0);
                upperIntake.set(0);
                Timer.delay(0.125);
                // lowerIntake.set(-power/2);
                // upperIntake.set(-power/2);
                // Timer.delay(0.07);
                // lowerIntake.set(0);
                // upperIntake.set(0);
                // Timer.delay(0.225);
            }
            lowerIntake.set(-power/3);
            upperIntake.set(-power/3);
            Timer.delay(0.05);
            stopI();
        });
    }

    public void stopI(){
        //executorService.shutdownNow();
        lowerIntake.set(0);
        upperIntake.set(0);
    }
    
    //vison
    public boolean optic(){
        return optical1.get();
    }

    //set speed
    public void setIntakeRaw(double speed){
        lowerIntake.set(speed);
        upperIntake.set(speed);
    }

    @Override
    public void periodic(){}
    
}