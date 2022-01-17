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

  DriveTrain m_driveTrain;
  DoubleSupplier m_throttle;
  DoubleSupplier m_turn;
  BooleanSupplier m_driveSwitch;
  public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn, BooleanSupplier driveSwitch) {
    this.m_driveTrain = driveTrain;
    this.m_throttle = throttle;
    this.m_turn = turn;
    this.m_driveSwitch = driveSwitch;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.allDrive(m_throttle.getAsDouble(), m_turn.getAsDouble(), true, m_driveSwitch.getAsBoolean());
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
