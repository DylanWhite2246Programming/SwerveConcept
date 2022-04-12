// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Power extends SubsystemBase {
  
  private ShuffleboardTab power = Shuffleboard.getTab("Power");
  //private NetworkTable powerTable = NetworkTableInstance.getDefault().getTable("Power");

  PowerDistribution PDP = new PowerDistribution(Ports.kPDPCANID, ModuleType.kRev);
  /** Creates a new Power. */
  public Power() {
    PDP.setSwitchableChannel(false);

    power.addNumber("Voltage", this::getVoltage)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min",0,"max",13));
    
    power.addNumber("Amp Draw", this::getAmpDraw)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min",0,"max",120));
    
    power.addNumber("Power", this::getPower);
    
    power.add(PDP)
      .withWidget(BuiltInWidgets.kPowerDistribution);
  }

  public void setLights(boolean on){PDP.setSwitchableChannel(on);}

  public double getVoltage(){return PDP.getVoltage();}
  public double getAmpDraw(){return PDP.getTotalCurrent();}
  public double getPower(){return PDP.getTotalPower();}
  public double getAmpDraw(int channel){return PDP.getCurrent(channel);}  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
