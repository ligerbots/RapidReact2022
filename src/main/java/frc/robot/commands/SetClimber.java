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
public class SetClimber extends SequentialCommandGroup {
  Climber m_climber;
  /** Creates a new SetClimber. */
  public SetClimber(Climber climber) {
    m_climber = climber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // set the finished boolean to false
    new CommandFinished(false),

    // lower the elevators to get unlatched
    new SetElevatorHeight(m_climber, Constants.ELEVATOR_MIN_HEIGHT),
    
    // rotates arm to leave spaces for the elevator to raise up
    new SetArmAngle(m_climber, Constants.ARM_ANGLE_FOR_ELEVATOR_CLEARANCE).alongWith(new SetElevatorHeight(m_climber, Constants.MID_RUNG, 
            Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_ASCEND, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_ASCEND)),

    // set the finished boolean to true
    new CommandFinished(true));
  }
}