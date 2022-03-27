// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetElevatorHeightTest extends CommandBase {
  /** Creates a new SetElevatorHeight. */
  Climber m_climber;
  String m_key;

  boolean m_goingToZero;

  double[] m_height;

  boolean[] m_hitLimitSwitch;
  boolean[] m_limitSwitchAlreadyPressed;

  boolean[] m_ressetEncoder;

  public SetElevatorHeightTest(Climber climber, String key) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_key = key;
    m_height = new double[2];
    m_hitLimitSwitch = new boolean[2];
    m_limitSwitchAlreadyPressed = new boolean[2];
    m_ressetEncoder = new boolean[2];
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double height = Units.inchesToMeters(SmartDashboard.getNumber(m_key, 0.0));
    m_height[0] = height;
    m_height[1] = height;
    m_goingToZero = (height == Constants.ELEVATOR_MIN_HEIGHT);
    m_hitLimitSwitch[0] = false;
    m_hitLimitSwitch[1] = false;
    m_ressetEncoder[0] = false;
    m_ressetEncoder[1] = false;
    m_climber.setElevatorHeight(0, m_height[0]);
    m_climber.setElevatorHeight(1, m_height[1]);
    SmartDashboard.putBoolean("finished", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // If we're going to Zero, we have to check things here and potentially change
    // the
    // requested elevator height.
    if (m_goingToZero) {
      // We're going to need to check each elevator independently, so we need a loop
      for (int i = 0; i <= 1; i++) {
        // if we're going to zero, we need to make sure the elevator hit the limit
        // switch
        // Note that there is a race condition here. In Climber.periodic, it checks for
        // the switch and
        // ifPressed, it sets the encoder value and also calls setElevatorHeight to 0
        // which will raise the
        // elevator slightly and release the switch.
        // I think Commands run before Subsystems, so we should be OK.

        // only check the limit switches when the elevator reads below certain height
        // if(m_climber.getElevatorHeight()[i] >
        // Constants.ELEVATOR_CHECKING_LIMIT_SWITCH_HEIGHT) continue;

        // Make sure that the switch is pressed twice in a row.
        boolean tempPressedCheck = m_climber.m_limitSwitch[i].isPressed() &&
            m_climber.getElevatorHeight()[i] < Constants.ELEVATOR_CHECKING_LIMIT_SWITCH_HEIGHT;
        m_hitLimitSwitch[i] = tempPressedCheck && m_limitSwitchAlreadyPressed[i];
        m_limitSwitchAlreadyPressed[i] = tempPressedCheck;

        // if the limit swsitch is not triggered, we need to reduce the set height to
        // keep it going down
        if (!m_hitLimitSwitch[i]) {
          // limit switch is not pressed yet.
          // did we reach zero?
          if (Math.abs(m_climber.getElevatorHeight()[i] - m_height[i]) < Constants.ELEVATOR_HEIGHT_TOLERANCE) {
            // We reached zero, but since the limit switch was not hit, we need to keep
            // going.
            // Lower the elevator height. We'll check next time through to see if the switch
            // isPressed.
            m_height[i] -= Units.inchesToMeters(0.5);
            m_climber.setElevatorHeight(i, m_height[i]);
          }
        }
        // If we've hit the limit switch twice, we need to set the encoder value to limit switch height
        // and then raise elevator to ELEVATOR_MIN_HEIGHT.
        if (m_hitLimitSwitch[i] && m_limitSwitchAlreadyPressed[i] && !m_ressetEncoder[i]) {
          m_climber.m_elevatorMotor[i].getEncoder().setPosition(Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT);
          m_climber.setElevatorHeight(i, 0.0);
          m_ressetEncoder[i] = true;
        }
      }
    }
    SmartDashboard.putNumber("m_height" + 0, Units.metersToInches(m_height[0]));
    SmartDashboard.putNumber("m_height" + 1, Units.metersToInches(m_height[1]));
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
      finished =  Math.abs(arr[0] - m_height[0]) < Constants.ELEVATOR_HEIGHT_TOLERANCE
        || Math.abs(arr[1] - m_height[1]) < Constants.ELEVATOR_HEIGHT_TOLERANCE;
    }
    SmartDashboard.putBoolean("finished", finished);
    return finished;
  }
}
