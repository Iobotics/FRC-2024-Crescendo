// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerScore extends SequentialCommandGroup {
  Intake intake;
  Shooter shooter;
  Arm arm;
  /** Creates a new SpeakerScore. */
  public SpeakerScore(Intake intake, Shooter shooter, Arm arm) {
    this.intake = intake;
    this.shooter = shooter;
    this.arm = arm;

    addRequirements(intake);
    addRequirements(shooter);
    addRequirements(arm);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArm(arm, 0),
      new InstantCommand(() -> shooter.setSSpeed(1)),
      new InstantCommand(() -> intake.setISpeed(0.5, true, false))
    );
  }
}
