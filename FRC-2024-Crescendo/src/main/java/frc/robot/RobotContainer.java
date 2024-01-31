package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.Shooting;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;


public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
    private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
    private final Joystick gamepad = new Joystick(OIConstants.kGamepad);

    /* Drive Controls */
    private final int translationAxis = joystick1.getYChannel();
    private final int strafeAxis = joystick1.getXChannel();
    private final int rotationAxis = joystick2.getXChannel();
    private double scalar = 1.5;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(joystick1, 1);
    //private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    private final JoystickButton mIConsume = new JoystickButton(gamepad, 1);
    private final JoystickButton mSConsume = new JoystickButton(gamepad, 2);
    private final JoystickButton mIEject = new JoystickButton(gamepad, 3);
    private final JoystickButton mSEject = new JoystickButton(gamepad, 4);
    private final JoystickButton shooting = new JoystickButton(gamepad, 5);
    private final JoystickButton pulse = new JoystickButton(gamepad, 6);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();

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

        mIConsume.onTrue(new InstantCommand(() -> intake.setISpeed(0.5, false, false)));
        mIConsume.onFalse(new InstantCommand(() -> intake.stop()));

        mSConsume.onTrue(new InstantCommand(() -> intake.setSSpeed(0.25)));
        mSConsume.onFalse(new InstantCommand(() -> intake.stop()));

        mIEject.onTrue(new InstantCommand(() -> intake.setISpeed(-0.25, false, false)));
        mIEject.onFalse(new InstantCommand(() -> intake.stop()));

        mSEject.onTrue(new InstantCommand(() -> intake.setSSpeed(-1.0)));
        mSEject.onFalse(new InstantCommand(() -> intake.stop()));

        pulse.onTrue(new InstantCommand(() -> intake.pulse(0.5, 4)));
        pulse.onTrue(new InstantCommand(() -> intake.stop()));

        shooting.onTrue(new Shooting(intake));

    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return AutoChooser.getSelected();
    }









}
