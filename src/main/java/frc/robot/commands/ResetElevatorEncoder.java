// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ResetElevatorEncoder extends CommandBase {
  /** Creates a new SetGoal. */
  Climber m_climber;
  double m_goalUnits;
  public ResetElevatorEncoder(Climber climber) {
    m_climber = climber; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.m_elevatorAscend[0].getEncoder().setPosition(0.0);
    m_climber.m_elevatorAscend[1].getEncoder().setPosition(0.0);
    m_climber.m_elevatorAscend[0].setGoal(0.0);
    m_climber.m_elevatorAscend[1].setGoal(0.0);

    m_climber.m_elevatorDescend[0].getEncoder().setPosition(0.0);
    m_climber.m_elevatorDescend[1].getEncoder().setPosition(0.0);
    m_climber.m_elevatorDescend[0].setGoal(0.0);
    m_climber.m_elevatorDescend[1].setGoal(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
