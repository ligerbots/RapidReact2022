// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DescendElevatorToLimitSwitch extends CommandBase {
  Climber m_climber;
  double m_height;

  double m_maxVel;
  double m_maxAcc;

  Command m_command;

  public DescendElevatorToLimitSwitch(Climber climber) {
    this(climber, 0.0, Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_DESCEND,
        Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_DESCEND);
  }

  public DescendElevatorToLimitSwitch(Climber climber, double height, final double MAX_VEL_METER_PER_SEC,
      final double MAX_ACC_METER_PER_SEC) {
    m_maxAcc = MAX_ACC_METER_PER_SEC;
    m_maxVel = MAX_VEL_METER_PER_SEC;
    m_climber = climber;
    m_height = height;
  }

  @Override
  public void initialize() {
    m_height = Constants.HEIGHT_RESET_LIMIT_SWITCH;
    m_command = new TrapezoidProfileCommand(new TrapezoidProfile(
        // Limit the max acceleration and velocity
        new TrapezoidProfile.Constraints(
            m_maxVel,
            m_maxAcc),
        // End at desired position in meters; implicitly starts at 0
        new TrapezoidProfile.State(m_height, 0),
        // initial position state
        new TrapezoidProfile.State(m_climber.getElevatorHeight()[0], 0)),
        // Pipe the profile state to the drive
        setpointState -> m_climber.setElevatorHeight(setpointState));

    CommandScheduler.getInstance().schedule(m_command);
  }

  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }
  
  @Override
  public void end(boolean interrupted){
    m_command.end(interrupted);
    m_climber.m_elevatorMotor[0].getEncoder().setPosition(Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT);
    m_climber.m_elevatorMotor[1].getEncoder().setPosition(Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT);
  }
}
