package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.ApriltagAlign;
import frc.robot.Commands.PresetWrist;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Swiffer;
import frc.robot.Subsystems.Vision;


public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
    private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
<<<<<<< HEAD
    private final Joystick gamepad = new Joystick(OIConstants.kGamepad);
 
=======
    //private final Joystick gamepad = new Joystick(OIConstants.kGamepad);
    private final Joystick logi = new Joystick(0);

>>>>>>> cbae52df5764a70f8089490b265f650d3f3e79ff
    /* Drive Controls */
    private final int translationAxis = joystick1.getYChannel();
    private final int strafeAxis = joystick1.getXChannel();
    private final int rotationAxis = joystick2.getXChannel();
    private double scalar = 1.5;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(logi, 5);
    // private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    // private final JoystickButton consume = new JoystickButton(joystick1, 2);
    // private final JoystickButton eject = new JoystickButton(joystick1, 3);


    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Vision vision = new Vision();
    private final Swiffer swiffer = new Swiffer();
    //private final Intake intake = new Intake();

    //Allows for Autos to be chosen in Shuffleboard
    SendableChooser<Command> AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
<<<<<<< HEAD
    //     //drivetrain
    //     swerve.setDefaultCommand(
    //         new TeleopSwerve(
    //             swerve, 
    //             () -> -joystick1.getRawAxis(translationAxis), 
    //             () -> -joystick1.getRawAxis(strafeAxis), 
    //             () -> -joystick2.getRawAxis(rotationAxis), 
    //             () -> false,
    //             scalar
    //         )
    //     );
=======
        //drivetrain
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -logi.getRawAxis(1), 
                () -> -logi.getRawAxis(0), 
                () -> -logi.getRawAxis(4), 
                () -> false,
                scalar
            )
        );
>>>>>>> cbae52df5764a70f8089490b265f650d3f3e79ff

    //     // Put the chooser on the dashboard
    //     SmartDashboard.putData(AutoChooser);

    //     // Configure the controller bindings
    //     configureBindings();
    // }

    // public void configureBindings() {
    //     /* Driver Buttons */
    //     zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

<<<<<<< HEAD
    //     new JoystickButton(joystick1, 8).onTrue(
    //         new ApriltagAlign(vision, 1));
=======
        // new JoystickButton(joystick1, 8).onTrue(
        //     new ApriltagAlign(vision, 1));
>>>>>>> cbae52df5764a70f8089490b265f650d3f3e79ff

    //     // consume.onTrue(new InstantCommand(() -> intake.setSpeed(0.5)));
    //     // consume.onFalse(new InstantCommand(() -> intake.stop()));
    //     // eject.onTrue(new InstantCommand(() -> intake.setSpeed(-1.0)));
    //     // eject.onFalse(new InstantCommand(() -> intake.stop()));

        // --- SWIFFER BUTTON CONTROLS --- //

        // === SWIFFER ===

        new JoystickButton(gamepad, 2).whileTrue(
            new RunCommand(()->swiffer.setPowerWrist(0.1), swiffer));

        new JoystickButton(gamepad, 2).whileFalse(
            new RunCommand(()->swiffer.stopWrist(), swiffer));

        new JoystickButton(gamepad, 3).whileTrue(
            new RunCommand(()->swiffer.setPowerWrist(-0.25), swiffer));

        new JoystickButton(gamepad, 3).whileFalse(
            new RunCommand(()->swiffer.stopWrist(), swiffer));

        new JoystickButton(gamepad, 1).onTrue(
            new PresetWrist(swiffer, -1));
        
        new JoystickButton(gamepad, 4).onTrue(
            new PresetWrist(swiffer, -4));

        
        new JoystickButton(gamepad, 5).whileTrue(
            new RunCommand(()->swiffer.setPowerRoller(1), swiffer));

        new JoystickButton(gamepad, 5).whileFalse(
            new RunCommand(()->swiffer.stopRoller(), swiffer));

        new JoystickButton(gamepad, 6).whileTrue(
            new RunCommand(()->swiffer.setPowerRoller(-1), swiffer));

        new JoystickButton(gamepad, 6).whileFalse(
            new RunCommand(()->swiffer.stopRoller(), swiffer));

        
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return AutoChooser.getSelected();
    }









}
