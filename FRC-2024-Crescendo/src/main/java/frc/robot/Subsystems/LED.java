// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.LarsonAnimation;

public class LED extends SubsystemBase {

    private CANdle candle;
    private LarsonAnimation larsonAnimation;
    private int LedCount; //Is it 104 or 106?
   // config.stripType = LEDStripType.RGB; // set the strip type to RGB
   // config.brightnessScalar = 0.5; // dim the LEDs to half brightness
   // candle.configAllSettings(config);

    public LED() {
        candle = new CANdle(OIConstants.kCandle, "canivore");
        LedCount = 8;
        larsonAnimation = new LarsonAnimation(255, 70, 5, 0, 0.5, LedCount, BounceMode.Front, 4, 0);
    }

    public void red() {
        candle.setLEDs(255,0,0);
    }

    public void blue() {
        candle.setLEDs(0, 0, 255);
    }

    public void green() {
        candle.setLEDs(97, 84, 41);
    }

    public void custom(int r, int g, int b){
        candle.setLEDs(r, g, b);
    }

    public void clear(){
        candle.setLEDs(0, 0, 0);
    }
    
    public void larson(){
        candle.animate(larsonAnimation);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // custom();
    }

}