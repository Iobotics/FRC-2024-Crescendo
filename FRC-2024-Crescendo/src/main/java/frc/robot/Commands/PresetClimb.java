// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class PresetClimb extends Command {

  Climber climber;
  double posL;
  double posR;

  /** Creates a new PresetClimb. */
  public PresetClimb(Climber climber, double posL, double posR) {
    this.climber = climber;
    this.posL = posL;
    this.posR = posR;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Climber", false);
    climber.climbPOS(posL, posR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Climber", true);
    climber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climber.isClimbWithinError(posL, posR, 0.1));
  }
}
