// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.Subsystems.Intake;
// import frc.robot.Subsystems.Roller;
// import frc.robot.Subsystems.Shooter;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class Passing extends ParallelCommandGroup {
//   Intake intake;
//   Roller roller;
//   Shooter shooter;
//   /** Creates a new Passing. */
//   public Passing(Intake intake, Roller roller, Shooter shooter) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     this.intake = intake;
//     this.roller = roller;
//     this.shooter = shooter;

//     addRequirements(intake);
//     addRequirements(roller);
//     addRequirements(shooter);

//     addCommands(
//       new InstantCommand(() -> roller.setPowerRoller(-0.3, true)),
//       new InstantCommand(() -> shooter.setSSpeed(-0.3)),
//       new InstantCommand(() -> intake.setISpeed(0.6, false, false))  
//     );
//   }
// }
