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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.ApriltagAlign;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.Vision;


public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    /* Controllers */
    private final Joystick joystick1 = new Joystick(OIConstants.kJoystick1);
    private final Joystick joystick2 = new Joystick(OIConstants.kJoystick2);
    //private final Joystick gamepad = new Joystick(OIConstants.kGamepad);
    private final Joystick logi = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = joystick1.getYChannel();
    private final int strafeAxis = joystick1.getXChannel();
    private final int rotationAxis = joystick2.getXChannel();
    private double scalar = 1.5;

    /* Driver Buttons */

    // private final JoystickButton zeroGyro = new JoystickButton(joystick1, 1);
    // private final JoystickButton autoAim = new JoystickButton(joystick2, 1);
    private final JoystickButton zeroGyro = new JoystickButton(logi, 5);
    // private final JoystickButton robotCentric = new JoystickButton(joystick2, 1);
    // private final JoystickButton consume = new JoystickButton(joystick1, 2);
    // private final JoystickButton eject = new JoystickButton(joystick1, 3);


    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Vision vision = new Vision(swerve);
    //private final Intake intake = new Intake();

    //Allows for Autos to be chosen in Shuffleboard
    SendableChooser<Command> autoChooser;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerve.configureAutoBuilder();
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

        // NamedCommands.registerCommand("exampleCommand", subsystem.exampleCommand);

        autoChooser = AutoBuilder.buildAutoChooser();
        // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the controller bindings
        configureBindings();
    }

    public void configureBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.setGyro(swerve.getEstYaw())));
        // new JoystickButton(joystick2,1 ).whileTrue();





        // new JoystickButton(joystick1, 2).whileTrue(
        //     new ApriltagAlign(vision, swerve, 0.4));
        // new JoystickButton(joystick1, 8).onTrue(
        //     new ApriltagAlign(vision, 1));

        // consume.onTrue(new InstantCommand(() -> intake.setSpeed(0.5)));
        // consume.onFalse(new InstantCommand(() -> intake.stop()));
        // eject.onTrue(new InstantCommand(() -> intake.setSpeed(-1.0)));
        // eject.onFalse(new InstantCommand(() -> intake.stop()));
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }









}
