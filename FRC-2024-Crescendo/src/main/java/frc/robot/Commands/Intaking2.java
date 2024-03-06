// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;

public class Intaking2 extends Command {
  /** Creates a new Intaking2. */
  Intake intake;
  Arm arm;
  double speed;
  boolean enabled;
  boolean direction;

  public Intaking2(Intake intake, Arm arm, double speed, boolean enabled, boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.arm = arm;
    this.speed = speed;
    this.enabled = enabled;
    this.direction = direction;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("done", false);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   intake.setISpeed(speed, enabled, direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new MoveArm(arm, -2.33);
    SmartDashboard.putBoolean("done", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.optic();
  }
}
