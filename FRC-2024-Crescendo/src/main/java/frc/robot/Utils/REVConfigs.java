package frc.robot.Utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

public final class REVConfigs {
    public static final class CancoderConfig {

        public static void applyCancoderConfig(CANcoder device) {
            CANcoderConfiguration cfg = new CANcoderConfiguration();

            cfg.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;

            device.getConfigurator().apply(cfg);
        }
    }

    public static final class DriveConfig {
        public static void applyDriveConfig(CANSparkMax device) {
            device.restoreFactoryDefaults();
            var pid = device.getPIDController();
            var enc = device.getEncoder();

            /** Swerve Angle Motor Configurations */
            /* Motor Inverts and Neutral Mode */
            device.setInverted(false);
            device.setIdleMode(null);

            /* Encoder Ratio and Velocity Calculation Parameters */
            enc.setVelocityConversionFactor(0);
            enc.setMeasurementPeriod(0);
            enc.setAverageDepth(0);

            /* Current Limiting */
            device.setSmartCurrentLimit(0);
            device.setSecondaryCurrentLimit(0);

            /* PID Config */
            pid.setP(0);
            pid.setI(0);
            pid.setD(0, 0);
            pid.setFF(0, 0);
        }
    }

    public static final class AngleConfig {
        public static void applyAngleConfig(CANSparkMax device) {
            device.restoreFactoryDefaults();
            var pid = device.getPIDController();
            var enc = device.getEncoder();

            /** Swerve Angle Motor Configurations */
            /* Motor Inverts and Neutral Mode */
            device.setInverted(false);
            device.setIdleMode(null);

            /* Gear Ratio and Wrapping Config */
            enc.setPositionConversionFactor(0);

            /* Current Limiting */
            device.setSmartCurrentLimit(0);
            device.setSecondaryCurrentLimit(0);

            /* PID Config */
            pid.setP(0);
            pid.setI(0);
            pid.setD(0, 0);
            pid.setFF(0, 0);

        }
    }
}