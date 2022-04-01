/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class FaceShootingTarget extends CommandBase {
    /**
     * Creates a new FaceShootingTarget.
     */
    DriveTrain m_robotDrive;
    DriveCommand m_driveCommand;
    Vision m_vision;
    double m_acceptableError;

    boolean m_oldOldCheck;
    boolean m_oldCheck;
    boolean m_check;

    boolean m_targetAcquired;
    double m_headingError;
    double m_headingTarget;

    double m_startTime;

    LigerTimer m_waitLED;

    public FaceShootingTarget(DriveTrain robotDrive, Vision vision, double acceptableError, DriveCommand driveCommand) {
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
        m_oldOldCheck = false;
        m_check = false;
        m_oldCheck = false;
        m_startTime = LigerTimer.time();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_targetAcquired) {
            m_headingError = m_robotDrive.getHeading() - m_headingTarget;
            while (m_headingError > 180.0)
                m_headingError -= 360.0;
            while (m_headingError < -180.0)
                m_headingError += 360.0;
            SmartDashboard.putNumber("shooter/HeadingError", m_headingError);
            System.out.println("FaceShooter headingError = " + m_headingError);

            m_check = Math.abs(m_headingError) < m_acceptableError && m_oldCheck;
            // System.out.format("FaceShootingTarget: %3.2f%n", initialAngleOffset);
            m_robotDrive.drive(0, -m_robotDrive.turnSpeedCalc(m_headingError), false);

            m_oldCheck = Math.abs(m_headingError) < m_acceptableError && m_oldOldCheck;
            m_oldOldCheck = Math.abs(m_headingError) < m_acceptableError;
        } else if (m_waitLED.hasElapsed() && m_vision.getStatus() && m_vision.getDistance() > 1.0) {
            m_targetAcquired = true;

            m_headingError = m_vision.getRobotAngle();
            double startAngle = m_robotDrive.getHeading();
            m_headingTarget = startAngle - m_headingError;
            System.out.format("FaceShooter acquired: heading = %3.1f visionAngle = %3.1f targetHeading = %3.2f%n",
                    startAngle, m_headingError, m_headingTarget);

            if (Math.abs(m_headingError) < m_acceptableError) {
                // we are already within the acceptable error, so short circuit the command
                m_check = true;
                return;
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.format("FaceShooter finished: currentHeading = %3.1f targetHeading = %3.2f%n",
                m_robotDrive.getHeading(), m_headingTarget);

        m_robotDrive.tankDriveVolts(0, 0);
        if (m_driveCommand != null) m_driveCommand.schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(m_headingError) < m_acceptableError && m_check)
                || (!m_targetAcquired && (LigerTimer.time() - m_startTime) > 0.5)
                || LigerTimer.time() - m_startTime > 3.0;
    }
}