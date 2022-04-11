// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotDimentions;

public class Drivetrain extends SubsystemBase {
  private final Translation2d frontRightLocation = new Translation2d(RobotDimentions.kDistanceBetweenSwerveModules, RobotDimentions.kDistanceBetweenSwerveModules);
  private final Translation2d frontLeftLocation = new Translation2d(-RobotDimentions.kDistanceBetweenSwerveModules, RobotDimentions.kDistanceBetweenSwerveModules);
  private final Translation2d backLeftLocation = new Translation2d(-RobotDimentions.kDistanceBetweenSwerveModules, -RobotDimentions.kDistanceBetweenSwerveModules);
  private final Translation2d backRightLocation = new Translation2d(RobotDimentions.kDistanceBetweenSwerveModules, -RobotDimentions.kDistanceBetweenSwerveModules);

  private final SwerveModule frontRight = new SwerveModule(Ports.kFRDriveMotorCANID, Ports.kFRSteerMotorCADID);
  private final SwerveModule frontLeft = new SwerveModule(Ports.kFLDriveMotorCADID, Ports.kFLSteerMotorCADID);
  private final SwerveModule backleft = new SwerveModule(Ports.kBLDriveMotorCADID, Ports.kBLSteerMotorCADID);
  private final SwerveModule backright = new SwerveModule(Ports.kBRDriveMotorCADID, Ports.kBRSteerMotorCADID);

  private final AHRS navx = new AHRS(Port.kMXP);

  private final SwerveDriveKinematics kinematics
    = new SwerveDriveKinematics(
      frontLeftLocation,
      frontRightLocation,
      backLeftLocation,
      backRightLocation
    );

  SwerveDriveOdometry odometry 
    = new SwerveDriveOdometry(kinematics, navx.getRotation2d().times(-1));

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    navx.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
