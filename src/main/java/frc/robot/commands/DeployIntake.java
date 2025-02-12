// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DeployIntake extends Command {
  /** Creates a new DeployIntake. */
  DriveTrain m_driveTrain;
  LigerTimer m_backwardsTime = new LigerTimer(0.1);
  LigerTimer m_forwardTime = new LigerTimer(0.1);
  boolean m_backwardsStarted= false;
  public DeployIntake(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = driveTrain;
    // addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.tankDriveVolts(-5.0, -5.0);
    m_forwardTime.start();
    m_backwardsStarted= false;
    System.out.println("deploy intake started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_forwardTime.hasElapsed() && !m_backwardsStarted) {
      m_driveTrain.tankDriveVolts(4.0, 4.0);
      m_backwardsTime.start();
      m_backwardsStarted = true;
      System.out.println("inverting direction");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.tankDriveVolts(0.0, 0.0);
    System.out.println("stopping deployIntake, interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_backwardsStarted && m_backwardsTime.hasElapsed();
  }
}
