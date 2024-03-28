package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import javax.management.InstanceAlreadyExistsException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
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
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Extension;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Wrist;
import pabeles.concurrency.ConcurrencyOps.NewInstance;


public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
    private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
    private final Joystick swifferGamepad = new Joystick(OIConstants.kSwifferGamepad);
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
    private final JoystickButton zeroGyro = new JoystickButton(joystick1, 8);

    private final JoystickButton armIntake = new JoystickButton(joystick2, 1);
    private final JoystickButton mIConsume = new JoystickButton(joystick2, 3);
    private final JoystickButton resetWheels = new JoystickButton(joystick1, 6);
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
    private final JoystickButton climberLUp = new JoystickButton(fight, 4); //green
    private final JoystickButton climberLDown = new JoystickButton(fight, 2); //red
    private final JoystickButton climberRUp = new JoystickButton(fight, 1); //green
    private final JoystickButton climberRDown = new JoystickButton(fight, 3); //red
    private final JoystickButton climberLock = new JoystickButton(fight, 6); //r1
    private final JoystickButton climberUnlock = new JoystickButton(fight, 5); //l1



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
    

    private ParallelCommandGroup PassPos = new ParallelCommandGroup(
        new MoveArm(arm, -4.5).withTimeout(3),
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
        new PresetExt(ext, -65).withTimeout(4),
        new PresetWrist(wrist, 22).withTimeout(3)
    );

    private SequentialCommandGroup AutoNotePickup = new SequentialCommandGroup(
            new InstantCommand(()-> arm.setArmPos(-21.2)).withTimeout(2),
            new ParallelCommandGroup(new AutoIntakeNote(vision.intakeCamera,swerve,intake), 
            new Intaking(intake, false, false)),
            new MoveArm(arm, 0),
            new InstantCommand(() -> intake.pulse(-0.5, 4)));
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.configureAutoBuilder();
        //drivetrain
        swerve.setDefaultCommand(teleopSwerve);
        swerve.resetModulesToAbsolute();
        arm.setDefaultCommand(new InstantCommand(() -> arm.brake(), arm));

        // NamedCommands.registerCommand("exampleCommand", subsystem.exampleCommand);

        autoChooser = AutoBuilder.buildAutoChooser();
        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putBoolean("arm", false);

        // Configure the controller bindings
        configureBindings();
    }

    public void configureBindings() {

        /* Driver Buttons */

        /* SUBSYSTEMS */

        plainSpeaker.onTrue(new MoveArm(arm, -18.5));

        resetWheels.onTrue(new InstantCommand(() -> swerve.resetModulesToAbsolute()));

        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro())); 

        autoIntakeNote.onTrue(AutoNotePickup);

        mIConsume.onTrue(
            new SequentialCommandGroup(
                new Intaking(intake, false, false),
                new ParallelCommandGroup(
                    new MoveArm(arm, 0),
                    new InstantCommand(() -> intake.pulse(-0.5, 4))
                )
            )
        );

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

        armIntake.onTrue(new InstantCommand(()-> arm.setArmPos(-22.0)).withTimeout(2));

        // speaker.onTrue(SpeakerScore);

        alignSpeaker.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> teleopRotationOverride.run()),
            new InstantCommand(()->arm.setArmPos(swerve.getShootingAngle()))));
        alignSpeaker.onFalse(new InstantCommand(() -> teleopRotationOverride.stop(true)));

        ampScore.onTrue(AmpScore);
        trapScore.onTrue(TrapScore);

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
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new PathPlannerAuto("red");
        // return autoChooser.getSelected();
    }
}
