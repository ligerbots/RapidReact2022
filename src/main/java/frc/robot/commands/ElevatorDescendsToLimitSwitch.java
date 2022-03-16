// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ElevatorDescendsToLimitSwitch extends CommandBase {

  Climber m_climber;
  boolean m_elevator0Pressed, m_elevator1Pressed;

  /** Creates a new ElevatorDescendsToLimitSwitch. */
  public ElevatorDescendsToLimitSwitch(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setElevatorHeight(Units.inchesToMeters(-2.0));
    m_elevator0Pressed = false;
    m_elevator1Pressed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_climber.m_limitSwitch[0].isPressed()) m_elevator0Pressed = true;
    if(m_climber.m_limitSwitch[1].isPressed()) m_elevator1Pressed = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator0Pressed && m_elevator1Pressed;
  }
}
