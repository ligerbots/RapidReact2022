// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */

  DriveTrain m_driveTrain;
  DoubleSupplier m_throttle;
  DoubleSupplier m_turn;

  double m_multiplier = Constants.FORWARD_BACKWARD;
  double m_rmutiplier = Constants.ROTATION;
  boolean m_slowMode = false;

  public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn) {
    m_driveTrain = driveTrain;
    m_throttle = throttle;
    m_turn = turn;

    SmartDashboard.putNumber("drivetrain/speedMultiplier", m_multiplier);
    SmartDashboard.putNumber("drivetrain/rotationMultiplier", m_rmutiplier);
    SmartDashboard.putBoolean("slowMode", m_slowMode);

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_slowMode = SmartDashboard.getBoolean("slowMode", false);

    if (m_slowMode) {
      m_multiplier = SmartDashboard.getNumber("drivetrain/speedMultiplier", 1);
      m_rmutiplier = SmartDashboard.getNumber("drivetrain/rotationMultiplier", 1);
      m_driveTrain.drive(m_throttle.getAsDouble() * m_multiplier, m_turn.getAsDouble() * m_rmutiplier, true);
    } else
      m_driveTrain.drive(m_throttle.getAsDouble(), m_turn.getAsDouble(), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
