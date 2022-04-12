// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.RobotDimentions;

public class Drivetrain extends SubsystemBase {

  private ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");
  //private NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drivetrain");

  private final double kMaxSpeed = 3;

  private double yCoef, xCoef, zCoef;

  private final Translation2d frontRightLocation 
    = new Translation2d(RobotDimentions.kDistanceBetweenSwerveModules, RobotDimentions.kDistanceBetweenSwerveModules);
  private final Translation2d frontLeftLocation 
    = new Translation2d(-RobotDimentions.kDistanceBetweenSwerveModules, RobotDimentions.kDistanceBetweenSwerveModules);
  private final Translation2d backLeftLocation 
    = new Translation2d(-RobotDimentions.kDistanceBetweenSwerveModules, -RobotDimentions.kDistanceBetweenSwerveModules);
  private final Translation2d backRightLocation 
    = new Translation2d(RobotDimentions.kDistanceBetweenSwerveModules, -RobotDimentions.kDistanceBetweenSwerveModules);

  private final SwerveModule frontRight = new SwerveModule(Ports.kFRDriveMotorCANID, Ports.kFRSteerMotorCADID, Ports.kFRCANCoderCANID);
  private final SwerveModule frontLeft = new SwerveModule(Ports.kFLDriveMotorCADID, Ports.kFLSteerMotorCADID, Ports.kFLCANCoderCANID);
  private final SwerveModule backleft = new SwerveModule(Ports.kBLDriveMotorCADID, Ports.kBLSteerMotorCADID, Ports.kBLCANCoderCANID);
  private final SwerveModule backright = new SwerveModule(Ports.kBRDriveMotorCADID, Ports.kBRSteerMotorCADID, Ports.kBRCANCoderCANID);

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

    drivetrainTab.addNumber("Y Velocity", this::getyVelocity);
    drivetrainTab.addNumber("Rot Velocity", this::getRotVelociy);

    drivetrainTab.addNumber("Adv Drive Motor Temp", ()->
      (frontRight.getDriveMotorTemp()+
      frontLeft.getDriveMotorTemp()+
      backleft.getDriveMotorTemp()+
      backright.getDriveMotorTemp())/4
    );
    drivetrainTab.addNumber("Adv Steer Motor Temp", ()->
      (frontRight.getSteerMotorTemp()+
      frontLeft.getSteerMotorTemp()+
      backleft.getSteerMotorTemp()+
      backright.getSteerMotorTemp())/4
    );

    drivetrainTab.addNumber("FR Drive Motor Temp", frontRight::getDriveMotorTemp);
    drivetrainTab.addNumber("FL Drive Motor Temp", frontLeft::getDriveMotorTemp);
    drivetrainTab.addNumber("BL Drive Motor Temp", backleft::getDriveMotorTemp);
    drivetrainTab.addNumber("BR Drive Motor Temp", backright::getDriveMotorTemp);

    navx.reset();
  }

  public void drive(double x, double y, double z, boolean fieldRelative) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xCoef*x, yCoef*y, zCoef*z, navx.getRotation2d().times(-1))
        : new ChassisSpeeds(xCoef*x, yCoef*y, zCoef*z)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backleft.setDesiredState(swerveModuleStates[2]);
    backright.setDesiredState(swerveModuleStates[3]);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(
      frontLeft.getState(),
      frontRight.getState(),
      backleft.getState(),
      backright.getState()
    );
  }
  
  public double getyVelocity(){return getChassisSpeeds().vyMetersPerSecond;}
  public double getRotVelociy(){return getChassisSpeeds().omegaRadiansPerSecond;}

  public void setCoef(double xCoe, double yCoe, double zCoe) {xCoef=xCoe; yCoef=yCoe; zCoef=zCoe;}

  @Override
  public void periodic() {
    odometry.update(
      navx.getRotation2d().times(-1), 
      frontLeft.getState(),
      frontRight.getState(),
      backleft.getState(),
      backright.getState()
    );
    // This method will be called once per scheduler run
  }
}
