// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdjustRobotAngle extends CommandBase {
  /** Creates a new AdjustRobotAngle. */

  DriveTrain m_driveTrain;
  double m_goalAngle;
  Command m_command;
  public AdjustRobotAngle(DriveTrain driveTrain, double goalAngle) {
    m_goalAngle = goalAngle;
    m_driveTrain = driveTrain;
  }

  @Override
  public void initialize(){

    double startDisLeft = m_driveTrain.getLeftEncoderDistance();
    double startDisRight = m_driveTrain.getRightEncoderDistance();

    boolean turnToLeft = m_goalAngle > Units.degreesToRadians(m_driveTrain.getHeading());
    double turnAngle = Math.abs(m_goalAngle - Units.degreesToRadians(m_driveTrain.getHeading()));

    Command m_command = new TrapezoidProfileCommand(
      // The motion profile to be executed
      new TrapezoidProfile(
          // The motion profile constraints
          new TrapezoidProfile.Constraints(Constants.DRIVETRAIN_MAX_VEL_METER_PER_SEC, Constants.DRIVETRAIN_MAX_ACC_METER_PER_SEC_SQ),
          // Goal state
          // use radians
          new TrapezoidProfile.State(m_driveTrain.disToTurn(turnAngle), 0),
          // Initial state
          new TrapezoidProfile.State(0.0, 0)),
      state -> m_driveTrain.setSetPoint(turnToLeft, state, startDisLeft, startDisRight));

      CommandScheduler.getInstance().schedule(m_command);
  }

  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
}
