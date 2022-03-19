// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetElevatorHeight extends CommandBase {
  /** Creates a new SetElevatorHeight. */
  Climber m_climber;
  double [] m_height;
  double m_tolerance;

  // flag to identify if we're going to Zero
  boolean m_goingToZero;

  // Did we hit the limit switch?
  boolean [] m_hitLimitSwitch;


  // Constructor with default tolerance
  public SetElevatorHeight(Climber climber, double height) {
    this(climber, height, Constants.ELEVATOR_HEIGHT_TOLERANCE);
  }

  public SetElevatorHeight(Climber climber, double height, double tolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_height = new double[2];
    m_height[0] = height;
    m_height[1] = height;
    m_tolerance = tolerance;

    m_goingToZero = (height == Constants.ELEVATOR_MIN_HEIGHT);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize each height
    m_climber.setElevatorHeight(0, m_height[0]);
    m_climber.setElevatorHeight(1, m_height[1]);
    m_hitLimitSwitch = new boolean[2];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If we're going to Zero, we have to check things here and potentially change the 
    // requested elevator height.
    if (m_goingToZero) {
      // We're going to need to check each elevator independently, so we need a loop
      for (int i = 0; i <= 1; i++) {
        // if we're going to zero, we need to make sure the elevator hit the limit switch
        // Note that there is a race condition here. In Climber.periodic, it checks for the switch and
        // ifPressed, it sets the encoder value and also calls setElevatorHeight to 0 which will raise the
        // elevator slightly and release the switch.
        // I think Commands run before Subsystems, so we should be OK.

        // only check the limit switches when the elevator reads below certain height
        if(m_climber.getElevatorHeight()[i] > Constants.ELEVATOR_CHECKING_LIMIT_SWITCH_HEIGHT) continue;

        if (m_climber.m_limitSwitch[i].isPressed()) {
          m_hitLimitSwitch[i] = true;
        } else {
          // limit switch is not pressed yet.
          // did we reach zero?
          if (Math.abs(m_climber.getElevatorHeight()[i] - m_height[i]) < Constants.ELEVATOR_HEIGHT_TOLERANCE) {
            // We reached zero, but since the limit switch was not hit, we need to keep going.
            // Lower the elevator height. We'll check next time through to see if the switch isPressed.
            m_height[i] -= Units.inchesToMeters(0.5);
            m_climber.setElevatorHeight(i, m_height[i]);
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
      finished = m_hitLimitSwitch[0] && m_hitLimitSwitch[1];
    } else {
      double[] arr = m_climber.getElevatorHeight();
      finished =  Math.abs(arr[0] - m_height[0]) < m_tolerance
        || Math.abs(arr[1] - m_height[1]) < m_tolerance;
    }
    return finished;
  }
}
