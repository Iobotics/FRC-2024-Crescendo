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
// import frc.robot.Commands.ApriltagAlign;
import frc.robot.Commands.Intaking;
import frc.robot.Commands.MoveArm;
import frc.robot.Commands.Passing;
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


public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
    private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
    private final Joystick swifferGamepad = new Joystick(OIConstants.kSwifferGamepad);
    private final Joystick gamepad = new Joystick(OIConstants.kGamepad);
    private final Joystick fight = new Joystick(OIConstants.kFight);

    /* Drive Controls */
    private final int translationAxis = joystick1.getYChannel();
    private final int strafeAxis = joystick1.getXChannel();
    private final int rotationAxis = joystick2.getXChannel();
    private double scalar = 1.5;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(gamepad, 7);
    //private final JoystickButton autoAim = new JoystickButton(joystick2, 1);
    //private final JoystickButton zeroGyro = new JoystickButton(joystick1, 1);
    //private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    //private final JoystickButton consume = new JoystickButton(joystick1, 2);
    //private final JoystickButton eject = new JoystickButton(joystick1, 3);
    //private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);

    private final JoystickButton mIConsume = new JoystickButton(fight, 2);
    private final JoystickButton mSConsume = new JoystickButton(fight, 3);
    private final JoystickButton mIEject = new JoystickButton(fight, 4);
    private final JoystickButton mSEject = new JoystickButton(fight, 1);
    //private final JoystickButton shooting = new JoystickButton(gamepad, 5);
    private final JoystickButton pulse = new JoystickButton(gamepad, 2);
    private final JoystickButton armUp = new JoystickButton(gamepad, 5);
    private final JoystickButton armDown = new JoystickButton(gamepad, 6);
    private final JoystickButton collapsing = new JoystickButton(gamepad, 1);
    //** gamepad 4 */
    private final JoystickButton armIntake = new JoystickButton(gamepad, 4);
    private final JoystickButton pass = new JoystickButton(fight, 6);
    // private final JoystickButton speaker = new JoystickButton(fight, 5);
    private final JoystickButton alignSpeaker = new JoystickButton(gamepad, 3);


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
        new MoveArm(arm, -2.2).withTimeout(3),
        new PresetWrist(wrist, 5).withTimeout(3),
        new PresetExt(ext, 1).withTimeout(3)
    );

    // private Command SpeakerScore = new RunCommand(() ->
    //     new MoveArm(arm, -7.00)
    // );

    private ParallelCommandGroup AmpScore = new ParallelCommandGroup(
        new PresetExt(ext, 14).withTimeout(3),
        new PresetWrist(wrist, 25).withTimeout(3)
    );



    //Allows for Autos to be chosen in Shuffleboard
    SendableChooser<Command> autoChooser;
    
    TeleopSwerve teleopSwerve = new TeleopSwerve(
                swerve, 
                () -> -gamepad.getRawAxis(1), 
                () -> -gamepad.getRawAxis(0), 
                () -> -gamepad.getRawAxis(4), 
                () -> false,
                scalar
            );

    public TeleopRotationOverride teleopRotationOverride = new TeleopRotationOverride(swerve::getRotationToSpeaker, swerve, teleopSwerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.configureAutoBuilder();
        //drivetrain
        swerve.setDefaultCommand(teleopSwerve);
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

        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro())); 
        mIConsume.onTrue(new SequentialCommandGroup(
            new Intaking(intake, false, false),
            new MoveArm(arm, -1.02),
            new InstantCommand(() -> intake.pulse(0.5, 4))
        ));
        //mIConsume.onFalse(new InstantCommand(() -> intake.stopI())); // fight 2

        mSConsume.onTrue(new InstantCommand(() -> shooter.setSSpeed(0.25)));
        mSConsume.onFalse(new InstantCommand(() -> shooter.stopS()));

        mIEject.onTrue(new InstantCommand(() -> intake.setISpeed(-0.25, false, false)));
        mIEject.onFalse(new InstantCommand(() -> intake.stopI())); // fight 4

        mSEject.onTrue(new InstantCommand(() -> shooter.setSSpeed(-1.0))); // fight 1
        mSEject.onFalse(new InstantCommand(() -> shooter.stopS()));

        pass.onTrue(new Passing(intake, roller, shooter)); // fight 6
        pass.onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> shooter.stopS()),
            new InstantCommand(() -> intake.stopI()),
            new InstantCommand(() -> roller.stopRoller())
        ));

        pulse.onTrue(new InstantCommand(() -> intake.pulse(0.5, 4)).withTimeout(1));
        //pulse.onTrue(new InstantCommand(() -> intake.stopI()));

        //shooting.onTrue(new Shooting(intake));

        armUp.onTrue(new InstantCommand(() -> swerve.zeroGyro())); // logi 5
        // armUp.onFalse(new InstantCommand(() -> arm.armSpeed(0)));

        // armDown.onTrue(new InstantCommand(() -> arm.armSpeed(-0.15))); // logi 6
        // armDown.onFalse(new InstantCommand(() -> arm.armSpeed(0)));

        collapsing.onTrue(PassPos.withTimeout(2));

        armIntake.onTrue(new InstantCommand(()-> arm.setArmPos(-9.6)).withTimeout(2));

        // speaker.onTrue(SpeakerScore);

        // alignSpeaker.whileTrue(new ParallelCommandGroup(
        //     new InstantCommand(() -> teleopRotationOverride.run()),
        //     new InstantCommand(()->arm.setArmPos(swerve.getShootingAngle()))));
        // alignSpeaker.onFalse(new InstantCommand(() -> teleopRotationOverride.stop(true)));

        alignSpeaker.onTrue(AmpScore);


        // new JoystickButton(fight, 9).whileTrue(
        //     new RunCommand(()->ext.setPowerArm(-fight.getY()/5), ext));

        new JoystickButton(fight, 9).whileTrue(
            new RunCommand(()->ext.setPowerArm(-fight.getY())));
                 
        new JoystickButton(fight, 9).onFalse(
            new InstantCommand(()->ext.stopArm(), ext)); 

        // new JoystickButton(fight, 10).whileTrue(
        //    new PresetWrist(wrist, 2.4));

        new JoystickButton(fight, 10).whileTrue(
            //new InstantCommand(()->ext.presetArm(13))
            //AmpScore
            new RunCommand(() -> wrist.setPowerWrist(fight.getY()/5), wrist)
        );
        new JoystickButton(fight, 10).onFalse(
            new InstantCommand(() -> wrist.setPowerWrist(0), wrist)
        );
                 
        // new JoystickButton(fight, 10).onFalse(
        //     new InstantCommand(()->ext.stopArm(), ext)); 

        // new JoystickButton(swifferGamepad, 3).onTrue(
        //     new PresetArm(ext, 0));
        
        // new JoystickButton(swifferGamepad, 4).onTrue(
        //     new PresetArm(ext, -20));

        new JoystickButton(fight, 8).onTrue(
            new RunCommand(()->roller.setPowerRoller(-0.75, false), roller));

        new JoystickButton(fight, 8).onFalse(
            new RunCommand(()->roller.stopRoller(), roller));

        new JoystickButton(fight, 7).onTrue(
            new RunCommand(()->roller.setPowerRoller(1.0, false), roller));

        new JoystickButton(fight, 7).onFalse(
            new RunCommand(()->roller.stopRoller(), roller));

        // new JoystickButton(gamepad, 3).whileTrue(
        //     new RunCommand(()->intake.setIntakeRaw(0.3), swiffer));

        // new JoystickButton(gamepad, 3).whileFalse(
        //     new RunCommand(()->intake.stopI()));

        new JoystickButton(fight, 5).onTrue(
            new InstantCommand(()->arm.setArmPos(-7)));

        new JoystickButton(fight, 5).onFalse(
            new InstantCommand(()->arm.stopA()));

        new JoystickButton(gamepad, 5).onTrue(
            new InstantCommand(()->climber.setPower(0.15)));
        
        new JoystickButton(gamepad, 5).onFalse(
            new InstantCommand(()->climber.stop()));

        new JoystickButton(gamepad, 6).onTrue(
            new InstantCommand(()->climber.setPower(-0.15)));
        
        new JoystickButton(gamepad, 6).onFalse(
            new InstantCommand(()->climber.stop()));
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
