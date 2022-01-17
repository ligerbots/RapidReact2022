// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */

  DriveTrain driveTrain;
  DoubleSupplier throttle;
  DoubleSupplier turn;
  BooleanSupplier driveSwitch;
  public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn, BooleanSupplier driveSwitch) {
    this.driveTrain = driveTrain;
    this.throttle = throttle;
    this.turn = turn;
    this.driveSwitch = driveSwitch;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.allDrive(throttle.getAsDouble(), turn.getAsDouble(), true, driveSwitch.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
