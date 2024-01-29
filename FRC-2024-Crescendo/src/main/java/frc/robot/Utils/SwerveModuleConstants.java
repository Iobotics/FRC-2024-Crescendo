// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final double CANoffsets;
    public final boolean invertedA;
    public final boolean invertedD;
  
    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleoffset2
     * @param inverted 
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleoffset2, double CANoffsets, boolean invertedA, boolean invertedD) {
      this.driveMotorID = driveMotorID;
      this.angleMotorID = angleMotorID;
      this.cancoderID = canCoderID;
      this.angleOffset = angleoffset2;
      this.CANoffsets = CANoffsets;
      this.invertedA = invertedA;
      this.invertedD = invertedD;
    }
  }