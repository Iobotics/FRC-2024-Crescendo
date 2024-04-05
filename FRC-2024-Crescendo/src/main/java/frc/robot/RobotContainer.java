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
    private final JoystickButton zeroGyro = new JoystickButton(joystick1, 8);
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

    // Operator Buttons
    private final JoystickButton passPosition = new JoystickButton(gamepad, 3);
    private final JoystickButton pass = new JoystickButton(gamepad, 2);
    private final JoystickButton amp = new JoystickButton(gamepad, 1);
    private final JoystickButton rollerShoot = new JoystickButton(gamepad, 4);
    private final JoystickButton spinUp = new JoystickButton(gamepad, 5);
    private final JoystickButton shoot = new JoystickButton(gamepad, 6);
    private final JoystickButton revSpin = new JoystickButton(gamepad, 7);
    private final JoystickButton revShoot = new JoystickButton(gamepad, 8);
    private final JoystickButton revRoller = new JoystickButton(gamepad, 10);
    private final JoystickButton wristShotPos = new JoystickButton(gamepad, 11);
    private final JoystickButton trapAssist = new JoystickButton(gamepad, 12);
    private final JoystickButton stowWristClimb = new JoystickButton(gamepad, 9);

    //Manual Buttons
    private final JoystickButton climberLUp = new JoystickButton(fight, 4); //green
    private final JoystickButton climberLDown = new JoystickButton(fight, 2); //red
    private final JoystickButton climberRUp = new JoystickButton(fight, 3); //green
    private final JoystickButton climberRDown = new JoystickButton(fight, 1); //red
    private final JoystickButton climberLock = new JoystickButton(fight, 5); //r1
    private final JoystickButton climberUnlock = new JoystickButton(fight, 6); //l1
    private final JoystickButton manualRollerIn = new JoystickButton(fight, 7);
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
    
    private Command stowWrist = new PresetWrist(wrist, 50);

    private ParallelCommandGroup PassPos = new ParallelCommandGroup(
        new MoveArm(arm, 0.39293).withTimeout(0.5),
        new PresetWrist(wrist, 0.9).withTimeout(3),
        new PresetExt(ext,0).withTimeout(3)
    ); 

    private ParallelCommandGroup AmpScore = new ParallelCommandGroup(
        new PresetExt(ext, -30).withTimeout(3),
        new PresetWrist(wrist, 25.9).withTimeout(3)
    );

    private ParallelCommandGroup TrapScore = new ParallelCommandGroup(
        new PresetExt(ext, -71.5).withTimeout(5),
        new PresetWrist(wrist, 26.76).withTimeout(3)
    );

    private SequentialCommandGroup AutonomousPickup = new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setSSpeed(0.05)).withTimeout(0.1),
        new InstantCommand(()-> arm.setArmPos(0.171)),
        new Intaking(intake, false, false),
        new ParallelCommandGroup(
            new InstantCommand(()-> arm.setArmPos(0.3)),
            new InstantCommand(() -> intake.pulse(-0.5, 4))
    ));

    private SequentialCommandGroup AutonomousSpeaker = new SequentialCommandGroup(
        new InstantCommand(()-> intake.checkContact()),
        new WaitCommand(0.05),
        new ParallelCommandGroup(
            new InstantCommand(()->arm.followSpeaker()),
            new InstantCommand(() -> shooter.setSSpeed(-1.0)),
            new WaitCommand(0.8)
        ),
        new InstantCommand(() -> intake.setIntakeRaw(-0.25)).withTimeout(1),
        new WaitCommand(0.3),
        new ParallelCommandGroup(
            new InstantCommand(() -> shooter.stopS(), shooter),
            new InstantCommand(() -> intake.stopI(), intake),
            new InstantCommand(()-> arm.stopFollowSpeaker())
        )
    );

    //Allows for Autos to be chosen in Shuffleboard
    private final SendableChooser<String> autoChooser;

    private final SendableChooser<String> alliance;
    
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

        NamedCommands.registerCommand("AutoPickup", AutonomousPickup);
        NamedCommands.registerCommand("AutonomousSpeaker", AutonomousSpeaker);
        NamedCommands.registerCommand("stowWrist", new PresetWrist(wrist, 50));

        NamedCommands.registerCommand("blue", new InstantCommand(() -> swerve.switchAlliance("blue")));
        NamedCommands.registerCommand("red", new InstantCommand(() -> swerve.switchAlliance("red")));
        // NamedCommands.registerCommand("VisionSpeaker", VisionSpeaker);

        // autoChooser = AutoBuilder.buildAutoChooser();
        // Put the chooser on the dashboard
        // SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putBoolean("arm", false);
        SmartDashboard.putBoolean("Climber", false);

        // Configure the controller bindings
        arm.configureSpeakerFollow(swerve::getShootingAngle);
        configureBindings();

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("center", "center");
        autoChooser.addOption("rightToLeft", "rightToLeft");
        autoChooser.addOption("ignoreMid1", "ignoreMid1");
        autoChooser.addOption("ignoreMid2", "ignoreMid2");
        autoChooser.addOption("none", "none");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        alliance = new SendableChooser<>();
        alliance.setDefaultOption("blue", "blue");
        alliance.addOption("red", "red");
        SmartDashboard.putData("Alliance", alliance);
    }

    public Swerve getSwerve(){
        return swerve;
    }

    public void configureBindings() {

    //     /* Driver Buttons */

        /* SUBSYSTEMS */

        stowWristClimb.onTrue(new PresetWrist(wrist, 30)); 

        plainSpeaker.onTrue(new MoveArm(arm, -18.5));

        resetWheels.onTrue(new InstantCommand(() -> swerve.resetModulesToAbsolute()));

        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro())); 

        manualRollerIn.onTrue(new RunCommand(()->roller.setPowerRoller(1.0,false), roller));
        manualRollerIn.onFalse(new RunCommand(()->roller.stopRoller(),roller));

        manualRollerOut.onTrue(new RunCommand(()->roller.setPowerRoller(-1.0,false), roller));
        manualRollerOut.onFalse(new RunCommand(()->roller.stopRoller(),roller));

        manualWristIn.onTrue(new InstantCommand(()->wrist.setPowerWrist(0.4)));
        manualWristIn.onFalse(new InstantCommand(()->wrist.stopWrist()));

        manualWristOut.onTrue(new InstantCommand(()->wrist.setPowerWrist(-0.4)));
        manualWristOut.onFalse(new InstantCommand(()->wrist.stopWrist()));

        mIConsume.whileTrue(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new MoveArm(arm, 0.17).withTimeout(0.5),
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

        mIConsume.onFalse(
            new ParallelCommandGroup(
                new MoveArm(arm, 0.452),
                new InstantCommand(() -> intake.stopI())
            )
        );

        pass.onTrue(new Passing(intake, roller, shooter)); // fight 6
        pass.onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> shooter.stopS()),
            new InstantCommand(() -> intake.stopI()),
            new InstantCommand(() -> roller.stopRoller())
        ));

        passPosition.onTrue(PassPos);

        amp.onTrue(AmpScore);

        rollerShoot.onTrue(new InstantCommand(() -> roller.setPowerRoller(-1, false)));
        rollerShoot.onFalse(new InstantCommand(() -> roller.stopRoller()));

        spinUp.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> intake.checkContact()),
            new WaitCommand(0.05),
            new InstantCommand(() -> shooter.setSSpeed(-1.0))
        ));
        spinUp.onFalse(new InstantCommand(() -> shooter.stopS()));

        shoot.onTrue(new InstantCommand(() -> intake.setISpeed(-0.5, false, false)));
        shoot.onFalse(new InstantCommand(() -> intake.stopI()));

        revSpin.onTrue(new InstantCommand(() -> shooter.setSSpeed(0.5)));
        revSpin.onFalse(new InstantCommand(() -> shooter.stopS()));

        revShoot.onTrue(new InstantCommand(() -> intake.setISpeed(0.3, false, false)));
        revShoot.onFalse(new InstantCommand(() -> intake.stopI()));

        revRoller.onTrue(new InstantCommand(() -> roller.setPowerRoller(1, false)));
        revRoller.onFalse(new InstantCommand(() -> roller.stopRoller()));

        wristShotPos.onTrue(stowWrist);

        trapAssist.onTrue(TrapScore);


        alignSpeaker.whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> teleopRotationOverride.run()),
            new InstantCommand(()->arm.followSpeaker())
        ));
        alignSpeaker.onFalse(new ParallelCommandGroup(
            new InstantCommand(()-> arm.stopFollowSpeaker()),
            new InstantCommand(() -> teleopRotationOverride.stop(true))
        ));

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
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return new PathPlannerAuto(autoChooser.getSelected());
    }
}
