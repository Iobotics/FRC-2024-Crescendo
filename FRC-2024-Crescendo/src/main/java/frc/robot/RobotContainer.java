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
    private final Joystick fight = new Joystick(OIConstants.kFight);

    /* Drive Controls */
    // private final int translationAxis = joystick1.getYChannel();
    // private final int strafeAxis = joystick1.getXChannel();
    // private final int rotationAxis = joystick2.getXChannel();
    private double scalar = 1.5;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(gamepad, 7);
    //private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    private final JoystickButton mIConsume = new JoystickButton(fight, 2);
    private final JoystickButton mSConsume = new JoystickButton(fight, 3);
    private final JoystickButton mIEject = new JoystickButton(fight, 4);
    private final JoystickButton mSEject = new JoystickButton(fight, 1);
    // private final JoystickButton shooting = new JoystickButton(gamepad, 5);
    private final JoystickButton pulse = new JoystickButton(gamepad, 2);
    private final JoystickButton armUp = new JoystickButton(gamepad, 5);
    private final JoystickButton armDown = new JoystickButton(gamepad, 6);
    private final JoystickButton setArm = new JoystickButton(gamepad, 1);
    private final JoystickButton setArm1 = new JoystickButton(gamepad, 4);

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
                () -> -gamepad.getRawAxis(1), 
                () -> -gamepad.getRawAxis(0), 
                () -> -gamepad.getRawAxis(4), 
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
        mIConsume.onFalse(new InstantCommand(() -> intake.stopI()));

        mSConsume.onTrue(new InstantCommand(() -> intake.setSSpeed(0.25)));
        mSConsume.onFalse(new InstantCommand(() -> intake.stopS()));

        mIEject.onTrue(new InstantCommand(() -> intake.setISpeed(-0.25, false, false)));
        mIEject.onFalse(new InstantCommand(() -> intake.stopI()));

        mSEject.onTrue(new InstantCommand(() -> intake.setSSpeed(-1.0)));
        mSEject.onFalse(new InstantCommand(() -> intake.stopS()));

        pulse.onTrue(new InstantCommand(() -> intake.pulse(0.5, 4)));
        pulse.onTrue(new InstantCommand(() -> intake.stopI()));

        //shooting.onTrue(new Shooting(intake));

        armUp.onTrue(new InstantCommand(() -> intake.armSpeed(0.15)));
        armUp.onFalse(new InstantCommand(() -> intake.armSpeed(0)));

        armDown.onTrue(new InstantCommand(() -> intake.armSpeed(-0.15)));
        armDown.onFalse(new InstantCommand(() -> intake.armSpeed(0)));

        setArm.onTrue(new InstantCommand(() -> intake.setArmPos(1.5)));
        setArm1.onTrue(new InstantCommand(()-> intake.setArmPos(3)));

    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return AutoChooser.getSelected();
    }

}
