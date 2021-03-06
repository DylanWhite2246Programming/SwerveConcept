// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class Ports{
        public static final int
            kFRDriveMotorCANID = 1,
            kFRSteerMotorCADID = 2,
            kFRCANCoderCANID   = 3,

            kFLDriveMotorCADID = 4,
            kFLSteerMotorCADID = 5,
            kFLCANCoderCANID   = 6,

            kBLDriveMotorCADID = 7,
            kBLSteerMotorCADID = 8,
            kBLCANCoderCANID = 9,
            
            kBRDriveMotorCADID = 10,
            kBRSteerMotorCADID = 11,
            kBRCANCoderCANID   = 12,

            kPDPCANID = 13;

    }
    public final class RobotDimentions{
        public static final double kDistanceBetweenSwerveModules = 0;
    }
}
