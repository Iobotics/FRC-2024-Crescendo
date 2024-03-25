// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Subsystems;

// import java.util.concurrent.CompletableFuture;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.SwifferConstants;

// /** Add your docs here. */
// public class Roller extends SubsystemBase{
//     private CANSparkMax roller;
//     private DigitalInput swifferIntake;

//     public Roller(){
//         //Declare channel for optical limit switch in the swiffer
//         swifferIntake = new DigitalInput(SwifferConstants.kDigitalInput);

//         roller = new CANSparkMax(SwifferConstants.kRoller, MotorType.kBrushless);

//         roller.setIdleMode(IdleMode.kBrake);

//         roller.restoreFactoryDefaults();

//         roller.setSmartCurrentLimit(20);
//     }  

//     // ---Roller Functions---//

//     //Set roller power function
//     public void setPowerRoller(double speed, boolean enabled){
//         if(enabled){
//         CompletableFuture.runAsync(() -> {
//             do{
//             roller.set(speed);
//             }while(isNoteDetected() == true);
//             Timer.delay(1);
//             stopRoller();
//         });
//         }
//         else if(!enabled){
//         roller.set(speed);
//         }
//     }

//     //Note detection function
//     public boolean isNoteDetected(){
//         return(swifferIntake.get());
//     }

//     //Stop roller function
//     public void stopRoller(){
//         roller.set(0);
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//         SmartDashboard.putBoolean("Note?", isNoteDetected()); //Update note intake status (FIX THIS)
//     }

// }
