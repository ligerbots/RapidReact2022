// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetElevatorHeight extends CommandBase {
  /** Creates a new SetElevatorHeight. */
  Climber m_climber;
  double [] m_height;

  // flag to identify if we're going to Zero
  boolean m_goingToZero;

  // Did we hit the limit switch?
  boolean [] m_hitLimitSwitch = {false, false};

  public SetElevatorHeight(Climber climber, double height) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_height[0] = height;
    m_height[1] = height;

    m_goingToZero = (height == Constants.ELEVATOR_MIN_HEIGHT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize each height
    m_climber.setElevatorHeight(0, m_height[0]);
    m_climber.setElevatorHeight(0, m_height[0]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if we're going to zero, we need to make sure both elevators hit the limit switch
    if (m_goingToZero) {
      // loop over both elevators
      for (int i = 0; i < 1; i++) {
        // first check to see if the limit switch was pressed
        if (m_climber.m_limitSwitch[i].isPressed()) {
          m_hitLimitSwitch[i] = true;
        } else {
          // limit switch is not pressed yet.
          // did we reach zero?
          if (Math.abs(m_climber.getElevatorHeight()[i] - m_height[i]) < Constants.ELEVATOR_HEIGHT_TOLERANCE) {
            // We reached zero, but since the limit switch was not hit, we need to keep going.
            m_height[i]
            m_climber.setElevatorHeight(i, m_height);
            m_loweredGoal
          }
        }
      }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = false;
    if (m_goingToZero) {
      finished = m_climber.m_limitSwitch[0].isPressed()
    }
    double[] arr = m_climber.getElevatorHeight();
    return Math.abs(arr[0] - m_height) < Constants.ELEVATOR_HEIGHT_TOLERANCE
    || Math.abs(arr[1] - m_height) < Constants.ELEVATOR_HEIGHT_TOLERANCE;
  }
}
