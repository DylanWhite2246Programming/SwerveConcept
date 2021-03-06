// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final NetworkTable limeTable = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry hasTarget,yaw,pitch,skew,area,ledMode,camMode,pipeline;
  /** Creates a new Limelight. */
  public Limelight() {
    hasTarget = limeTable.getEntry("tv");
    yaw = limeTable.getEntry("tx");
    pitch = limeTable.getEntry("ty");
    skew = limeTable.getEntry("ts");
    area = limeTable.getEntry("ta");

    ledMode = limeTable.getEntry("ledMode");
    camMode = limeTable.getEntry("camMode");
    pipeline = limeTable.getEntry("pipeline");
  }

  public boolean hasTargets(){return (int)hasTarget.getNumber(0)==1;}
  public double getYaw(){return yaw.getDouble(0);}
  public double getPitch(){return pitch.getDouble(0);}
  public double getSkew() {return skew.getDouble(0);}
  public double getArea(){return area.getDouble(0);}
  
  /**
   * @param mode 0 Auto, 1 off, 2 blink, 3 force on
   */
  public void setLedMode(int mode){ledMode.setNumber(mode);}
  public void setDriveMode(boolean drivermode){camMode.setNumber(drivermode?1:0);}
  public void setPipeline(int pipe){pipeline.setNumber(pipe);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
