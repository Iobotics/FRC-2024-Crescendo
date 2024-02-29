// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swiffer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Passing extends SequentialCommandGroup {
  Intake intake;
  Swiffer swiffer;
  Shooter shooter;
  /** Creates a new Passing. */
  public Passing() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
         new InstantCommand(() -> swiffer.setPowerRoller(0.3)),
        new InstantCommand(() -> shooter.setSSpeed(-0.3))
      ),
      new InstantCommand(() -> intake.setISpeed(0.5, false, false))
    );
  }
}
