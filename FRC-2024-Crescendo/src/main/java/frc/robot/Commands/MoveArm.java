// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  double pos;
  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.pos = pos;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmPos(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopA();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmWithinError(pos, 0.2);
  }
}
