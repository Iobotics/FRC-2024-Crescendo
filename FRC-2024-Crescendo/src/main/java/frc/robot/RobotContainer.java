package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;


public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
    private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
    //private final Joystick gamepad = new Joystick(OIConstants.kGamepad);

    /* Drive Controls */
    private final int translationAxis = joystick1.getYChannel();
    private final int strafeAxis = joystick1.getXChannel();
    private final int rotationAxis = joystick2.getXChannel();
    private double scalar = 1.5;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(joystick1, 1);
    private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    // private final JoystickButton consume = new JoystickButton(joystick1, 2);
    // private final JoystickButton eject = new JoystickButton(joystick1, 3);


    /* Subsystems */
    private final Swerve swerve = new Swerve();
    //private final Intake intake = new Intake();

    //Allows for Autos to be chosen in Shuffleboard
    SendableChooser<Command> AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //drivetrain
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -joystick1.getRawAxis(translationAxis), 
                () -> -joystick1.getRawAxis(strafeAxis), 
                () -> -joystick2.getRawAxis(rotationAxis), 
                () -> false,
                scalar
            )
        );

        // Put the chooser on the dashboard
        SmartDashboard.putData(AutoChooser);

        // Configure the controller bindings
        configureBindings();
    }

    public void configureBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        // consume.onTrue(new InstantCommand(() -> intake.setSpeed(0.5)));
        // consume.onFalse(new InstantCommand(() -> intake.stop()));
        // eject.onTrue(new InstantCommand(() -> intake.setSpeed(-1.0)));
        // eject.onFalse(new InstantCommand(() -> intake.stop()));
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return AutoChooser.getSelected();
    }









}
