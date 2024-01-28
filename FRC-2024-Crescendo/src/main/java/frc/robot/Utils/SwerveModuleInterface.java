package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleInterface {
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);

    public Rotation2d getCANcoder();

    public void resetToAbsolute();

    public SwerveModuleState getState();

    public int getModuleNumber();

    public SwerveModulePosition getPosition();

    public double getAbsolutePosition();
}