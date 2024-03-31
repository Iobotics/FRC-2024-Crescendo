package frc.robot;

import java.time.Instant;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Commands.AutoIntakeNote;
// import frc.robot.Commands.ApriltagAlign;
import frc.robot.Commands.Intaking;
import frc.robot.Commands.MoveArm;
import frc.robot.Commands.Passing;
import frc.robot.Commands.PresetClimb;
import frc.robot.Commands.PresetExt;
import frc.robot.Commands.PresetWrist;
import frc.robot.Commands.TeleopRotationOverride;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.gotoGoal;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Wrist;


public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
    private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
    private final Joystick gamepad = new Joystick(OIConstants.kGamepad);
    private final Joystick fight = new Joystick(OIConstants.kFight);

    /* Drivetrain Controls */
    private final int translationAxis = joystick1.getYChannel();
    private final int strafeAxis = joystick1.getXChannel();
    private final int rotationAxis = joystick2.getXChannel();
    private double scalar = 1.5;

    /* Driver Buttons */
    private final JoystickButton alignSpeaker = new JoystickButton(joystick1, 1);
    private final JoystickButton autoIntakeNote = new JoystickButton(joystick1, 2);
    private final JoystickButton plainSpeaker = new JoystickButton(joystick1, 3);
    // private final JoystickButton alignAmp = new JoystickButton(joystick1, 1);
    private final JoystickButton startClimb = new JoystickButton(joystick1, 6);
    private final JoystickButton finishClimb = new JoystickButton (joystick1, 7);
    private final JoystickButton zeroGyro = new JoystickButton(joystick1, 8);

    private final JoystickButton armIntake = new JoystickButton(joystick2, 3);
    private final JoystickButton mIConsume = new JoystickButton(joystick2, 1);
    private final JoystickButton resetWheels = new JoystickButton(joystick1, 6);
    private final JoystickButton alignToRightStage = new JoystickButton(joystick1, 7);

    private final JoystickButton cancelAutoSwerveCommands = new JoystickButton(joystick1, 5);

    //private final JoystickButton autoAim = new JoystickButton(joystick2, 1);
    //private final JoystickButton zeroGyro = new JoystickButton(joystick1, 1);
    //private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    //private final JoystickButton consume = new JoystickButton(joystick1, 2);
    //private final JoystickButton eject = new JoystickButton(joystick1, 3);
    //private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);

    /* Operator Buttons */
    private final JoystickButton pass = new JoystickButton(gamepad, 1);
    private final JoystickButton collapsing = new JoystickButton(gamepad, 2);
    private final JoystickButton ampScore = new JoystickButton(gamepad, 3);
    private final JoystickButton rollerShoot = new JoystickButton(gamepad, 4);
    private final JoystickButton mSEject = new JoystickButton(gamepad, 5);
    private final JoystickButton mIEject = new JoystickButton(gamepad, 6);
    private final JoystickButton defaultSpeaker = new JoystickButton(gamepad, 7);
    private final JoystickButton trapScore = new JoystickButton(gamepad, 10);
    private final JoystickButton manualSwiffer = new JoystickButton(gamepad, 8);
    private final JoystickButton climbFirst = new JoystickButton(gamepad, 9);

    private final JoystickButton climberLUp = new JoystickButton(fight, 4); //green
    private final JoystickButton climberLDown = new JoystickButton(fight, 2); //red
    private final JoystickButton climberRUp = new JoystickButton(fight, 1); //green
    private final JoystickButton climberRDown = new JoystickButton(fight, 3); //red
    private final JoystickButton climberLock = new JoystickButton(fight, 6); //r1
    private final JoystickButton climberUnlock = new JoystickButton(fight, 5); //l1
    private final JoystickButton manualRollerOut = new JoystickButton(fight, 8);
    private final JoystickButton manualWristOut = new JoystickButton(fight, 9);
    private final JoystickButton manualWristIn = new JoystickButton(fight, 10);


    //private final JoystickButton mSConsume = new JoystickButton(fight, 3);
    //private final JoystickButton shooting = new JoystickButton(gamepad, 5);
    //private final JoystickButton pulse = new JoystickButton(gamepad, 2);
    // private final JoystickButton armUp = new JoystickButton(gamepad, 5);
    // private final JoystickButton armDown = new JoystickButton(gamepad, 6);

    //** gamepad 4 */
    // private final JoystickButton speaker = new JoystickButton(fight, 5);


    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Vision vision = new Vision(swerve);
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Arm arm = new Arm();
    private final Extension ext = new Extension();
    private final Wrist wrist = new Wrist();
    
    private final Roller roller = new Roller();
    private final Climber climber = new Climber();
    


    private Command stowWrist = new PresetWrist(wrist, 50.0);

    private SequentialCommandGroup TrapPos = new SequentialCommandGroup(
        new InstantCommand(() -> climber.unlock()),
        new PresetClimb(climber, -108.4, -103.4),
        new PresetWrist(wrist, 50.0)
    );

    private ParallelCommandGroup PassPos = new ParallelCommandGroup(
        new MoveArm(arm, 0.39293).withTimeout(0.5),
        new PresetWrist(wrist, 0.9).withTimeout(3),
        new PresetExt(ext,0).withTimeout(3)
    ); 

    // private Command SpeakerScore = new RunCommand(() ->
    //     new MoveArm(arm, -7.00)
    // );

    private ParallelCommandGroup AmpScore = new ParallelCommandGroup(
        new PresetExt(ext, -30).withTimeout(3),
        new PresetWrist(wrist, 25.9).withTimeout(3)
    );

    private ParallelCommandGroup TrapScore = new ParallelCommandGroup(
        new PresetExt(ext, -71.5).withTimeout(5),
        new PresetWrist(wrist, 25.666).withTimeout(3)
    );

    private SequentialCommandGroup Pullup = new SequentialCommandGroup(
        new PresetClimb(climber, 0, -1.5),
        new InstantCommand(() -> climber.lock()),
        TrapScore
    );

    private SequentialCommandGroup AutoNotePickup = new SequentialCommandGroup(
            new InstantCommand(()-> arm.setArmPos(-20.8)).withTimeout(2),
            new ParallelCommandGroup(new AutoIntakeNote(vision,swerve,intake), 
            new Intaking(intake, false, false)),
            new MoveArm(arm, 0.17),
            new InstantCommand(() -> intake.pulse(-0.5, 4)));

    private Command AutonomousIntake = new InstantCommand(()-> arm.setArmPos(0.17)).withTimeout(2);

    private SequentialCommandGroup AutonomousPickup = new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setSSpeed(0.05)).withTimeout(0.1),
        new InstantCommand(() -> arm.setArmPos(-22.0)).withTimeout(0.5),
        new Intaking(intake, false, false),
        new ParallelCommandGroup(
            new MoveArm(arm, -15.5),
            new InstantCommand(() -> intake.pulse(-0.5, 4))),
        new ParallelCommandGroup(
            new InstantCommand(() -> intake.setIntakeRaw(0.1)).withTimeout(0.15),
            new InstantCommand(() -> shooter.setSSpeed(0.1)).withTimeout(0.15))
            );

    private SequentialCommandGroup AutonomousSpeaker = new SequentialCommandGroup(
        new MoveArm(arm, -18.5).withTimeout(0.5),
        new InstantCommand(() -> shooter.setSSpeed(-1.0)),
        new WaitCommand(1),
        new InstantCommand(() -> intake.setIntakeRaw(-1)).withTimeout(1),
        new InstantCommand(() -> shooter.stopS(), shooter),
        new InstantCommand(() -> intake.stopI(), intake));

    private SequentialCommandGroup AutonomousSpeaker1 = new SequentialCommandGroup(
        new MoveArm(arm, -14).withTimeout(2),
        new InstantCommand(() -> shooter.setSSpeed(-1.0)),
        new WaitCommand(1),
        new InstantCommand(() -> intake.setIntakeRaw(-1)).withTimeout(1),
        new InstantCommand(() -> shooter.stopS(), shooter),
        new InstantCommand(() -> intake.stopI(), intake));

    private SequentialCommandGroup AutonomousSpeaker2 = new SequentialCommandGroup(
        new MoveArm(arm, -13.4).withTimeout(2),
        new InstantCommand(() -> shooter.setSSpeed(-1.0)).withTimeout(1),
        new WaitCommand(1),
        new InstantCommand(() -> intake.setIntakeRaw(-1)).withTimeout(1),
        new InstantCommand(() -> shooter.stopS(), shooter),
        new InstantCommand(() -> intake.stopI(), intake));


    private SequentialCommandGroup AutoSpeakerScore = new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setSSpeed(-1.0)),
        new RunCommand(()->arm.setArmPos(swerve.getShootingAngle())).withTimeout(1.5),
        new WaitCommand(1.0),
        new InstantCommand(() -> intake.setISpeed(-0.25, false, false)),
        new InstantCommand(() -> shooter.stopS()),
        new InstantCommand(() -> intake.stopI())
    );

    private Command goToStageRight = new gotoGoal(VisionConstants.redStageRight,swerve);



    // private ParallelCommandGroup ClimberUp = new ParallelCommandGroup(
    //     new SequentialCommandGroup(
    //         new InstantCommand(()->climber.unlock()),
    //         new InstantCommand(()->climber.climbPOS(15))),
    //     new PresetExt(ext, 0.75),
    //     new PresetWrist(wrist, 25)
    // );

    // private ParallelCommandGroup ClimberDown = new ParallelCommandGroup(
    //     new SequentialCommandGroup(
    //         new PresetClimb(climber, 0),
    //         new InstantCommand(()->climber.lock())),
    //     new ParallelCommandGroup(TrapScore)
    // );



    //Allows for Autos to be chosen in Shuffleboard
    SendableChooser<Command> autoChooser;
    
    TeleopSwerve teleopSwerve = new TeleopSwerve(
                swerve, 
                () -> -joystick1.getRawAxis(translationAxis), 
                () -> -joystick1.getRawAxis(strafeAxis), 
                () -> -joystick2.getRawAxis(rotationAxis), 
                () -> false,
                scalar
            );

    public TeleopRotationOverride teleopRotationOverride = new TeleopRotationOverride(swerve::getRotationToSpeaker, swerve, teleopSwerve);

    // private SequentialCommandGroup VisionSpeaker = new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         new InstantCommand(() -> teleopRotationOverride.run()),
    //         new InstantCommand(() -> arm.setArmPos(swerve.getShootingAngle())).withTimeout(1)),
    //     new ParallelCommandGroup( 
    //         new InstantCommand(() -> shooter.setSSpeed(-1.0)).withTimeout(1),
    //         new WaitCommand(1),
    //         new InstantCommand(() -> intake.setIntakeRaw(-1))),
    //     new ParallelCommandGroup(
    //         new InstantCommand(() -> shooter.stopS(), shooter),
    //         new InstantCommand(() -> intake.stopI(), intake)
    //     ));


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.configureAutoBuilder();
        //drivetrain
        swerve.setDefaultCommand(teleopSwerve);
        swerve.resetModulesToAbsolute();
        arm.setDefaultCommand(new InstantCommand(() -> arm.brake(), arm));

        // NamedCommands.registerCommand("exampleCommand", subsystem.exampleCommand);
        NamedCommands.registerCommand("AutoIntakeNote", AutoNotePickup);
        NamedCommands.registerCommand("AutoSpeakerScore", AutoSpeakerScore);
        NamedCommands.registerCommand("AutonomousIntake", AutonomousIntake);
        NamedCommands.registerCommand("AutonomousPickup", AutonomousPickup);
        NamedCommands.registerCommand("AutonomousSpeaker", AutonomousSpeaker);
        NamedCommands.registerCommand("AutonomousSpeaker1", AutonomousSpeaker1);
        NamedCommands.registerCommand("AutonomousSpeaker2", AutonomousSpeaker2);
        // NamedCommands.registerCommand("VisionSpeaker", VisionSpeaker);

        // autoChooser = AutoBuilder.buildAutoChooser();
        // Put the chooser on the dashboard
        // SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putBoolean("arm", false);

        // Configure the controller bindings
        configureBindings();
    }

    public void configureBindings() {

        /* Driver Buttons */

        /* SUBSYSTEMS */

        climbFirst.onTrue(TrapPos);

        plainSpeaker.onTrue(new MoveArm(arm, -18.5));

        resetWheels.onTrue(new InstantCommand(() -> swerve.resetModulesToAbsolute()));

        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro())); 

        autoIntakeNote.onTrue(AutoNotePickup);

        alignToRightStage.onTrue(goToStageRight);
        alignToRightStage.onFalse(new InstantCommand(() -> goToStageRight.cancel()));

        cancelAutoSwerveCommands.onTrue(new InstantCommand(()->AutoNotePickup.cancel()));

        manualRollerOut.onTrue(new RunCommand(()->roller.setPowerRoller(-1.0,false),roller));
        manualRollerOut.onFalse(new RunCommand(()->roller.stopRoller(),roller));

        manualWristIn.onTrue(new InstantCommand(()->wrist.setPowerWrist(0.4)));
        manualWristIn.onFalse(new InstantCommand(()->wrist.stopWrist()));

        manualWristOut.onTrue(new InstantCommand(()->wrist.setPowerWrist(-0.4)));
        manualWristOut.onFalse(new InstantCommand(()->wrist.stopWrist()));

        mIConsume.onTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new MoveArm(arm, 0.171).withTimeout(0.5),
                    new InstantCommand(() -> shooter.setSSpeed(0.01)),
                    new Intaking(intake, false, false)
                ),
                new ParallelCommandGroup(
                    new MoveArm(arm, 0.452),
                    //0.452
                    new InstantCommand(() -> intake.pulse(-0.5, 4)),
                    new InstantCommand(() -> shooter.stopS())
                )
            )
        );

        new JoystickButton(gamepad, 9).onTrue(stowWrist.withTimeout(2.5));

        //mIConsume.onFalse(new InstantCommand(() -> intake.stopI())); // fight 2

        // mSConsume.onTrue(new InstantCommand(() -> shooter.setSSpeed(0.25)));
        // mSConsume.onFalse(new InstantCommand(() -> shooter.stopS()));

        mIEject.onTrue(new InstantCommand(() -> 
            intake.setISpeed(-0.25, false, false)
            // ext.setPowerArm(0.25)
            //wrist.setPowerWrist(0.25)
            ));
        mIEject.onFalse(new InstantCommand(() -> 
            intake.stopI()
            // ext.stopArm()
            //wrist.stopWrist()
        )); // fight 4

        mSEject.onTrue(new InstantCommand(() -> 
            shooter.setSSpeed(-1.0)
            // ext.setPowerArm(-0.25)
            //wrist.setPowerWrist(-0.25)
        )); // fight 1
        mSEject.onFalse(new InstantCommand(() -> 
            shooter.stopS()
            // ext.stopArm()
            //wrist.stopWrist()
        ));

        pass.onTrue(new Passing(intake, roller, shooter)); // fight 6
        pass.onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> shooter.stopS()),
            new InstantCommand(() -> intake.stopI()),
            new InstantCommand(() -> roller.stopRoller())
        ));

        defaultSpeaker.onTrue(new MoveArm(arm, -8.5));

        
        // pulse.onTrue(new InstantCommand(() -> intake.pulse(0.5, 4)).withTimeout(1));
        //pulse.onTrue(new InstantCommand(() -> intake.stopI()));

        // speaker.onTrue(new MoveArm(arm, -9.0));

        //shooting.onTrue(new Shooting(intake));

        // armUp.onTrue(new InstantCommand(() -> swerve.zeroGyro())); // logi 5
        // // armUp.onFalse(new InstantCommand(() -> arm.armSpeed(0)));

        // armDown.onTrue(new InstantCommand(() -> swerve.resetModulesToAbsolute()));

        // armDown.onTrue(new InstantCommand(() -> arm.armSpeed(-0.15))); // logi 6
        // armDown.onFalse(new InstantCommand(() -> arm.armSpeed(0)));

        collapsing.onTrue(PassPos.withTimeout(2));

        armIntake.onTrue(new MoveArm(arm, 0.171).withTimeout(2)); //-21.4

        // speaker.onTrue(SpeakerScore);

        // alignSpeaker.whileTrue(new ParallelCommandGroup(
        //     new InstantCommand(() -> teleopRotationOverride.run()),
        //     new InstantCommand(() -> arm.setSupplier(swerve::getShootingAngle)),
        //     new InstantCommand(()->arm.setArmFollow())));

        alignSpeaker.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> teleopRotationOverride.run()),
            new MoveArm(arm ,swerve.getShootingAngle())
        ));
        alignSpeaker.onFalse(new InstantCommand(() -> teleopRotationOverride.stop(true)));

        ampScore.onTrue(AmpScore);
        trapScore.onTrue(Pullup);

        manualSwiffer.onTrue(new InstantCommand(()-> ext.setPowerArm(gamepad.getY())));
        manualSwiffer.onFalse(new InstantCommand(()-> ext.stopArm()));

        climberLUp.onTrue(new InstantCommand(() -> climber.setPower(0.5)));
        climberLUp.onFalse(new InstantCommand(() -> climber.setPower(0)));
        climberLDown.onTrue(new InstantCommand( () -> climber.setPower(-0.5)));
        climberLDown.onFalse(new InstantCommand(() -> climber.setPower(0)));

        climberRUp.onTrue(new InstantCommand(() -> climber.setPower(0.5)));
        climberRUp.onFalse(new InstantCommand(() -> climber.setPower(0)));
        climberRDown.onTrue(new InstantCommand( () -> climber.setPower(-0.5)));
        climberRDown.onFalse(new InstantCommand(() -> climber.setPower(0)));

        climberLock.onTrue(new InstantCommand(() -> climber.lock()));
        climberUnlock.onTrue(new InstantCommand(() -> climber.unlock()));

        // new JoystickButton(fight, 3).onTrue(
        //     new InstantCommand(()->climber.zeroRatchets()));

        // new JoystickButton(fight, 9).whileTrue(
        //     new RunCommand(()->ext.setPowerArm(-fight.getY())));
                 
        // new JoystickButton(fight, 9).onFalse(
        //     new InstantCommand(()->ext.stopArm(), ext)); 

        // new JoystickButton(fight, 10).whileTrue(
        //    new PresetWrist(wrist, 2.4));

        // new JoystickButton(fight, 10).whileTrue(
        //     //new InstantCommand(()->ext.presetArm(13))
        //     //AmpScore
        //     new RunCommand(() -> wrist.setPowerWrist(fight.getY()/5), wrist)
        // );
        // new JoystickButton(fight, 10).onFalse(
        //     new InstantCommand(() -> wrist.setPowerWrist(0), wrist)
        // );
                 
        // new JoystickButton(fight, 10).onFalse(
        //     new InstantCommand(()->ext.stopArm(), ext)); 

        // new JoystickButton(swifferGamepad, 3).onTrue(
        //     new PresetArm(ext, 0));
        
        // new JoystickButton(swifferGamepad, 4).onTrue(
        //     new PresetArm(ext, -20));

        rollerShoot.onTrue(new RunCommand(()->roller.setPowerRoller(-1, false), roller));

        rollerShoot.onFalse(new RunCommand(()->roller.stopRoller(), roller));

        // new JoystickButton(fight, 7).onTrue(
        //     new RunCommand(()->roller.setPowerRoller(1.0, false), roller));

        // new JoystickButton(fight, 7).onFalse(
        //     new RunCommand(()->roller.stopRoller(), roller));

        // new JoystickButton(gamepad, 3).whileTrue(
        //     new RunCommand(()->intake.setIntakeRaw(0.3), swiffer));

        // new JoystickButton(gamepad, 3).whileFalse(
        //     new RunCommand(()->intake.stopI()));

        // new JoystickButton(fight, 5).onTrue(
        //     new InstantCommand(()->arm.setArmPos(-7)));

        // new JoystickButton(fight, 5).onFalse(
        //     new InstantCommand(()->arm.stopA()));

        // new JoystickButton(joystick2, 6).onTrue(
        //     new InstantCommand(()->climber.setPower(0.20)));
        
        // new JoystickButton(joystick2, 6).onFalse(
        //     new InstantCommand(()->climber.stopClimber()));

        // new JoystickButton(joystick2, 7).onTrue(
        //     new InstantCommand(()->climber.setPower(-0.20)));
        
        // new JoystickButton(joystick2, 7).onFalse(
        //     new InstantCommand(()->climber.stopClimber()));

        new JoystickButton(joystick2, 11).onTrue(
            new InstantCommand(()->climber.setPowerL(0.30)));
        
        new JoystickButton(joystick2, 11).onFalse(
            new InstantCommand(()->climber.stopL()));

        new JoystickButton(joystick2, 10).onTrue(
            new InstantCommand(()->climber.setPowerL(-0.30)));
        
        new JoystickButton(joystick2, 10).onFalse(
            new InstantCommand(()->climber.stopL()));

        new JoystickButton(joystick2, 7).onTrue(
            new InstantCommand(()->climber.setPowerR(0.30)));
        
        new JoystickButton(joystick2, 7).onFalse(
            new InstantCommand(()->climber.stopR()));

        new JoystickButton(joystick2, 6).onTrue(
            new InstantCommand(()->climber.setPowerR(-0.30)));
        
        new JoystickButton(joystick2, 6).onFalse(
            new InstantCommand(()->climber.stopR()));
        
        new JoystickButton(joystick2, 8).onTrue(
            new InstantCommand(()->climber.lock()));

        new JoystickButton(joystick2, 9).onTrue(
            new InstantCommand(()->climber.unlock()));

        new JoystickButton(joystick1, 11).onTrue(
            new PresetClimb(climber, -99, 100));
        
        new JoystickButton(joystick1, 10).onTrue(
            new PresetClimb(climber, 1, 5));

        // new JoystickButton(joystick1, 6).onTrue(
        //     new PresetClimb(climber, 104));

        // new JoystickButton(joystick1, 7).onTrue(
        //     new PresetClimb(climber, 1));

        // new JoystickButton(gamepad,6).onTrue(
        //     new InstantCommand(()->arm.armSpeed()));
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new PathPlannerAuto("Penguino");
        // return autoChooser.getSelected();
    }
}
