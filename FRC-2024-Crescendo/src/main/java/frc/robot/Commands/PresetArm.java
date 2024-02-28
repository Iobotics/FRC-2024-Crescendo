// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.Swiffer;

// public class PresetArm extends Command {

//   private Swiffer swiffer;
//   private double targetPos;
  
//   /** Creates a new PresetArm. */
//   public PresetArm(Swiffer swiffer, double targetPos) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     swiffer = this.swiffer;
//     addRequirements(swiffer);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     SmartDashboard.putBoolean("Arm Reached?", false);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     swiffer.presetArm(targetPos);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     SmartDashboard.putBoolean("Arm Reached?", true);
//     swiffer.stopWrist();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return swiffer.isArmWithinError(targetPos, 0.05);
//   }
// }
