// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Passing extends SequentialCommandGroup {
  Intake intake;
  Roller roller;
  Wrist wrist;
  Shooter shooter;
  Arm arm;
  /** Creates a new Passing. */
  public Passing(Intake intake, Roller roller, Wrist wrist, Shooter shooter, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.intake = intake;
    this.roller = roller;
    this.wrist = wrist;
    this.shooter = shooter;
    this.arm = arm;

    addRequirements(intake);
    addRequirements(roller);
    addRequirements(wrist);
    addRequirements(shooter);
    addRequirements(arm);

    addCommands(
      //new ParallelCommandGroup(
        new MoveArm(arm, -2.33),
        //new PresetWrist(wrist, 2.48)
      //),
      new ParallelCommandGroup(
        new InstantCommand(() -> roller.setPowerRoller(0.3, true)),
        new InstantCommand(() -> shooter.setSSpeed(-0.3)),
        new InstantCommand(() -> intake.setISpeed(0.6, false, false))  
      ), 
      new ParallelCommandGroup(
        new InstantCommand(() -> shooter.stopS()),
        new InstantCommand(() -> intake.stopI())
      )
    );
    
  }
}
