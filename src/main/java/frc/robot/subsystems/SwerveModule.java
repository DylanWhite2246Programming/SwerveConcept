// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {

  private WPI_TalonFX driveMotor, steerMotor;
  private CANCoder steerEncoder;

  private static final double kWheelCircumfrance = Units.inchesToMeters(4)*Math.PI;
  private static final double kGearRatio = 6.75;
  private static final int kCPR = 2048;

  //private static final double kSteeringRatio = 12.8;

  private final PIDController driveController
    = new PIDController(1, 0, .1);
  private final ProfiledPIDController steerController
    = new ProfiledPIDController(
      1, 
      0, 
      .1, 
      new TrapezoidProfile.Constraints(
        1, //vel
        1 //accel
      )
    );

  //TODO change
  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward steerFeedForward = new SimpleMotorFeedforward(1, .5);

  /** Creates a new ExampleSubsystem. */
  public SwerveModule(
    int driveMotorCANID, 
    int steerMotorCANID,
    int CANCoderCANID
  ) {
    driveMotor = new WPI_TalonFX(driveMotorCANID);
    steerMotor = new WPI_TalonFX(steerMotorCANID);
    steerEncoder = new CANCoder(CANCoderCANID);

    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    //steerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  public double getWheelDisplacement(){
    return (driveMotor.getSelectedSensorPosition()*kWheelCircumfrance)
    /(kCPR*kGearRatio);
  }
  public Rotation2d getTurnPosition(){
    return new Rotation2d(
      steerEncoder.getAbsolutePosition()
    );
    //return new Rotation2d(
    //  (steerMotor.getSelectedSensorPosition()*2*Math.PI)
    //  /(kCPR*kSteeringRatio)
    //);
  }

  public double getWheelVelocity(){
    return (driveMotor.getSelectedSensorVelocity()*10*kWheelCircumfrance)
      /(kCPR*kGearRatio);
  }
  public double getSteerRate(){
    return Units.degreesToRadians(steerEncoder.getVelocity());
    //return (steerMotor.getSelectedSensorVelocity()*10*2*Math.PI)
    //  /(kCPR*kSteeringRatio);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getWheelVelocity(), getTurnPosition());
  }

  public double getDriveMotorTemp(){return driveMotor.getTemperature();}
  public double getSteerMotorTemp() {return steerMotor.getTemperature();}

  public void setDesiredState(SwerveModuleState desiredState){
    SwerveModuleState state 
      = SwerveModuleState.optimize(desiredState, getTurnPosition());
    final double driveOutput
      = driveController.calculate(getWheelVelocity(), state.speedMetersPerSecond);

    final double driveFeedForward = this.driveFeedForward.calculate(state.speedMetersPerSecond);

    final double steerOutput
      = steerController.calculate(getTurnPosition().getRadians(), state.angle.getRadians());

    final double steerFeedForward
      = this.steerFeedForward.calculate(steerController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput+driveFeedForward);
    steerMotor.setVoltage(steerOutput+steerFeedForward);
  }

}
