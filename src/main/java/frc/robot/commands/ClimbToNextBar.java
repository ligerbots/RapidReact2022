// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToNextBar extends SequentialCommandGroup {
  /** Creates a new ClimbToNextBar. */
  Climber m_climber;
  public ClimbToNextBar(Climber climber) {

    m_climber = climber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // rotate the arm to point elevator towards the next bar
      new SetArmAngle(m_climber, Constants.ARM_ANGLE_TO_NEXT_BAR),

      // extend the elevator
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_MAX_HEIGHT),

      // parallel command group, to have the arm rotate while elevator retracts
      new RetractElevatorAdjustArm(m_climber),

      // rotate the arm to leave the previous bar and get to the side of the next bar
      new SetArmAngle(m_climber, 45.0),

      // fully retract the elevator
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_MIN_HEIGHT),

      // rotate the arm back to grab on to the bar
      new SetArmAngle(m_climber, 90.0));
  }
}