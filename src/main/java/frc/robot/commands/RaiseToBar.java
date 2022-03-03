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
public class RaiseToBar extends SequentialCommandGroup {
  /** Creates a new RaiseToBar. */
  Climber m_climber;
  public RaiseToBar(Climber climber) {
    m_climber = climber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(      
      // retract the elevator
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_MIN_HEIGHT),

      // rotates arm to grab on to the bar
      new SetArmAngle(m_climber, Constants.ARM_GRAB_THE_BAR));
  }
}	