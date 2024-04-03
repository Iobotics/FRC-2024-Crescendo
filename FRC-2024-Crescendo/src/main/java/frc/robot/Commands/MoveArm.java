// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class MoveArm extends Command {
  Arm arm;
  double pos;
  PIDController inController;
  PIDController outController;
  Supplier<Double> armPositionSupplier;
  boolean goingInwards;

  /** Creates a new MoveArm. */
  public MoveArm(Arm arm, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.pos = pos;
    this.inController = new PIDController(0.1,0.01,0.015);
    this.outController = new PIDController(8e-3,0.0,0.0001);

    if (this.arm.getArmPos() < this.pos) {
      goingInwards = true;
    }
    addRequirements(arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (goingInwards) {
    //   arm.armSpeed(inController.calculate(arm.getArmPos(),pos));
    // }
    // else {
    //   arm.armSpeed(outController.calculate(arm.getArmPos(),pos));
    // }

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
    return arm.isArmWithinError(pos, 0.001);
  }
}
