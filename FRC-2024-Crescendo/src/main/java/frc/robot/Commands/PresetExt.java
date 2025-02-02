// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Extension;

public class PresetExt extends Command {

  private Extension ext;
  private double targetPos;
  
  /** Creates a new PresetArm. */
  public PresetExt(Extension ext, double targetPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ext = ext;
    this.targetPos = targetPos;

    addRequirements(ext);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Arm Reached?", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ext.presetArm(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Arm Reached?", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ext.isArmWithinError(targetPos, 0.05);
  }
}
