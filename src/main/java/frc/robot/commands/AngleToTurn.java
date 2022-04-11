/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AngleToTurn extends CommandBase {
  /**
   * Creates a new FaceShootingTarget.
   */
  // double startingAngle;
  DriveTrain m_robotDrive;
  DriveCommand m_driveCommand;
  Vision m_vision;
  double m_acceptableError;
  Command m_turnCommand;
  boolean m_onTarget;
  boolean m_targetAcquired;
  double m_headingError;
  double m_startTime;
  LigerTimer m_waitLED;

  public AngleToTurn(DriveTrain robotDrive, Vision vision, double acceptableError, DriveCommand driveCommand) {
    m_robotDrive = robotDrive;
    m_acceptableError = acceptableError;
    m_driveCommand = driveCommand;
    m_vision = vision;
    m_waitLED = new LigerTimer(0.25);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    if (m_driveCommand != null) m_driveCommand.cancel();
    m_waitLED.start();

    m_vision.setMode(Vision.VisionMode.HUBFINDER);

    m_targetAcquired = false;
    m_onTarget = false;
    m_startTime = LigerTimer.time();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (m_targetAcquired) {
        // flip the sign TODO: test it
        m_turnCommand = new AdjustRobotAngle(m_robotDrive, -m_headingError);
        CommandScheduler.getInstance().schedule(m_turnCommand);
      } else if (m_vision.getStatus() && m_vision.getDistance() > 1.0 && m_waitLED.hasElapsed()) {
        m_targetAcquired = true;

        m_headingError = m_vision.getRobotAngle();
        System.out.format("FaceShooter acquired: visionAngle = %3.1f %n", m_headingError);

        if (Math.abs(m_headingError) < m_acceptableError) {
          // we are already within the acceptable error, so short circuit the command
          m_onTarget = true;
          return;
        }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotDrive.tankDriveVolts(0, 0);
    if (m_driveCommand != null) m_driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_onTarget 
        || m_turnCommand.isFinished() 
        || LigerTimer.time() - m_startTime > 3.0;
  }
}