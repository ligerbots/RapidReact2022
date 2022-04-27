/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AngleToTurn extends CommandBase {
    /**
     * Creates a new AngleToTurn.
     */
    DriveTrain m_robotDrive;
    DriveCommand m_driveCommand;
    Vision m_vision;
    double m_acceptableError;

    enum State {
        WAITING_ON_LED, WAITING_ON_VISION, STARTING_TURN, RUNNING, FINISHED
    }

    State m_state;

    // trapezoid command to do the actual turning
    Command m_turnCommand;
    // angle error as found from vision
    double m_headingError;
    // timer to wait for LED to turn on and vision to start getting good images
    LigerTimer m_waitLED = new LigerTimer(0.25);
    // general timeout for the whole command
    LigerTimer m_timeout = new LigerTimer(3.0);

    public AngleToTurn(DriveTrain robotDrive, Vision vision, double acceptableError, DriveCommand driveCommand) {
        m_robotDrive = robotDrive;
        m_vision = vision;
        m_acceptableError = acceptableError;
        m_driveCommand = driveCommand;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_driveCommand != null)
            m_driveCommand.cancel();

        m_vision.setMode(Vision.VisionMode.HUBFINDER);
        m_waitLED.start();

        m_state = State.WAITING_ON_LED;
        m_turnCommand = null;
        m_timeout.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putString("AngleToTurn/state", m_state.toString());

        switch (m_state) {
            case WAITING_ON_LED:
                if (m_waitLED.hasElapsed()) {
                    m_state = State.WAITING_ON_VISION;
                    // fall through
                } else
                    break;

            case WAITING_ON_VISION:
                if (m_vision.getStatus() && m_vision.getDistance() > 1.0) {
                    m_headingError = m_vision.getRobotAngle();
                    System.out.format("FaceShooter acquired: visionAngle = %3.1f %n", m_headingError);
                    SmartDashboard.putNumber("AngleToTurn/headingError", m_headingError);

                    if (Math.abs(m_headingError) < m_acceptableError)
                        m_state = State.FINISHED;
                    else
                        m_state = State.STARTING_TURN;
                }
                break;

            case STARTING_TURN:
                m_turnCommand = new AdjustRobotAngle(m_robotDrive, -m_headingError);
                CommandScheduler.getInstance().schedule(m_turnCommand);
                m_state = State.RUNNING;
                break;

            case RUNNING:
                if (m_turnCommand.isFinished())
                    m_state = State.FINISHED;
                break;

            case FINISHED:
                // nothing to do
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_robotDrive.tankDriveVolts(0, 0);
        if (m_driveCommand != null)
            m_driveCommand.schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_state == State.FINISHED || m_timeout.hasElapsed();
    }
}