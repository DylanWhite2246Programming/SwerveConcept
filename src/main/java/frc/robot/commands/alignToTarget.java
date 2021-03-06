// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignToTarget extends PIDCommand {
  /** Creates a new alignToTarget. */
  public alignToTarget(Drivetrain drivetrain, Limelight limelight, DoubleSupplier x, DoubleSupplier y) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, .1),
        // This should return the measurement
        limelight::getYaw,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {//figure out if this should be true
          drivetrain.drive(x.getAsDouble(), y.getAsDouble(), output, false, true);
        });
      addRequirements(drivetrain, limelight);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
