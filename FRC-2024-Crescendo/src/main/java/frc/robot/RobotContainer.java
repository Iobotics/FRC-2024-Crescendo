package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.AmpScore;
import frc.robot.Commands.ApriltagAlign;
import frc.robot.Commands.MoveArm;
import frc.robot.Commands.Passing;
import frc.robot.Commands.PresetArm;
import frc.robot.Commands.PresetWrist;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Arm;
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
    private final JoystickButton setArm = new JoystickButton(gamepad, 1);
    private final JoystickButton setArm1 = new JoystickButton(gamepad, 4);
    private final JoystickButton mSEjectA = new JoystickButton(fight, 6);
    private final JoystickButton setArmSpeaker = new JoystickButton(fight, 5);


    /* Subsystems */
    private final Swerve swerve = new Swerve();
    // private final Vision vision = new Vision(swerve);
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Arm arm = new Arm();
    private final Extension ext = new Extension();
    private final Wrist wrist = new Wrist();
    private final Roller roller = new Roller();
    

    private ParallelCommandGroup Collapse = new ParallelCommandGroup(
        new MoveArm(arm, -2.33),
        new PresetWrist(wrist, 2.48)
    );

    //Allows for Autos to be chosen in Shuffleboard
    SendableChooser<Command> autoChooser;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.configureAutoBuilder();
        //drivetrain
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -gamepad.getRawAxis(1), 
                () -> -gamepad.getRawAxis(0), 
                () -> -gamepad.getRawAxis(4), 
                () -> false,
                scalar
            )
        );

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
        mIConsume.onTrue(new InstantCommand(() -> intake.setISpeed(0.5, true, true)).withTimeout(1));
        mIConsume.onFalse(new InstantCommand(() -> intake.stopI()));

        mSConsume.onTrue(new InstantCommand(() -> shooter.setSSpeed(0.25)));
        mSConsume.onFalse(new InstantCommand(() -> shooter.stopS()));

        mIEject.onTrue(new InstantCommand(() -> intake.setISpeed(-0.25, false, false)));
        mIEject.onFalse(new InstantCommand(() -> intake.stopI()));

        mSEject.onTrue(new InstantCommand(() -> shooter.setSSpeed(-1.0)));
        mSEject.onFalse(new InstantCommand(() -> shooter.stopS()));

        mSEjectA.onTrue(new ParallelCommandGroup(
            new InstantCommand(() -> intake.setISpeed(0.6, false, false)),
            new InstantCommand(() -> roller.setPowerRoller(0.3, true)),
            new InstantCommand(() -> shooter.setSSpeed(-0.3))
        ));
        mSEjectA.onFalse(new ParallelCommandGroup(
            new InstantCommand(() -> shooter.stopS()),
            new InstantCommand(() -> intake.stopI()),
            new InstantCommand(() -> roller.stopRoller())
        ));

        pulse.onTrue(new InstantCommand(() -> intake.pulse(0.5, 4)).withTimeout(1));
        //pulse.onTrue(new InstantCommand(() -> intake.stopI()));

        //shooting.onTrue(new Shooting(intake));

        armUp.onTrue(new InstantCommand(() -> arm.armSpeed(0.15)));
        armUp.onFalse(new InstantCommand(() -> arm.armSpeed(0)));

        armDown.onTrue(new InstantCommand(() -> arm.armSpeed(-0.15)));
        armDown.onFalse(new InstantCommand(() -> arm.armSpeed(0)));

        // setArm.onTrue(new InstantCommand(() -> arm.setArmPos(1.5)));
        // setArm1.onTrue(new InstantCommand(()-> arm.setArmPos(1.75)));
        setArm.onTrue(new InstantCommand(()-> arm.setArmPos(-2.33)));
        setArm1.onTrue(new InstantCommand(()-> arm.setArmPos(-9.8)));


        setArmSpeaker.onTrue(new Passing(intake, roller, wrist, shooter, arm));
        // setArmSpeaker.onFalse(new ParallelCommandGroup(
        //     new InstantCommand(() -> shooter.stopS()),
        //     new InstantCommand(() -> intake.stopI())
        // ));

        // new JoystickButton(fight, 9).whileTrue(
        //     new RunCommand(()->ext.setPowerArm(-fight.getY()/5), ext));

        new JoystickButton(fight, 9).whileTrue(
            new RunCommand(()->ext.presetArm(13)));
                 
        new JoystickButton(fight, 9).whileFalse(
            new RunCommand(()->ext.stopArm(), ext)); 

        // new JoystickButton(fight, 10).whileTrue(
        //     new RunCommand(()->ext.setPowerWrist(fight.getY()/5), ext));

        new JoystickButton(fight, 10).whileTrue(
            new RunCommand(()->wrist.presetWrist(24)));
                 
        new JoystickButton(fight, 10).whileFalse(
            new RunCommand(()->wrist.stopWrist(), wrist)); 

        // new JoystickButton(swifferGamepad, 3).onTrue(
        //     new PresetArm(ext, 0));
        
        // new JoystickButton(swifferGamepad, 4).onTrue(
        //     new PresetArm(ext, -20));

        new JoystickButton(fight, 8).onTrue(
            new RunCommand(()->roller.setPowerRoller(0.75, false), roller));

        new JoystickButton(fight, 8).onFalse(
            new RunCommand(()->roller.stopRoller(), roller));

        new JoystickButton(fight, 7).whileTrue(
            new RunCommand(()->roller.setPowerRoller(-1.0, false), roller));

        new JoystickButton(fight, 7).whileFalse(
            new RunCommand(()->roller.stopRoller(), roller));

        // new JoystickButton(gamepad, 3).whileTrue(
        //     new RunCommand(()->intake.setIntakeRaw(0.3), swiffer));

        // new JoystickButton(gamepad, 3).whileFalse(
        //     new RunCommand(()->intake.stopI()));
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
