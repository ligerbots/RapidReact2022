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
public class ResetClimber extends SequentialCommandGroup {
  Climber m_climber;
  /** Creates a new SetClimber. */
  public ResetClimber(Climber climber) {
    m_climber = climber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // set the finished boolean to false
    new CommandFinished(false),

    // let the elevator goes all the way down to get resetted
    new DescendElevatorToLimitSwitch(m_climber),
    
    // ready the arm and elevators to be latched
    new SetArmAngle(m_climber, Constants.ARM_OFFSET_RAD).alongWith(new SetElevatorHeight(m_climber, Constants.ELEVATOR_OFFSET_METER)),

    // set the finished boolean to true
    new CommandFinished(true));
  }
}
