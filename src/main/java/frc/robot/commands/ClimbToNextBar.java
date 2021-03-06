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
      new CommandFinished(false),

      // rotate the arm to point elevator towards the next bar and extend the elevator
      // new SetArmAngle(m_climber, Constants.ARM_ANGLE_TO_NEXT_BAR).alongWith(new SetElevatorHeight(m_climber, Constants.ELEVATOR_MAX_HEIGHT)),
      new SetArmAngle(m_climber, Constants.ARM_ANGLE_TO_NEXT_BAR),
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_MAX_HEIGHT, 
          Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_ASCEND, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_ASCEND),

      // clear the command
      // CommandGroupBase.clearGroupedCommand(Command),

      // rotate the robot to make the elevator touch the bar first
      new SetArmAngle(m_climber, Constants.ARM_ROTATION_ELEVATOR_TOUCH_BAR),

      // retract elevator to secure it on the bar
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_HEIGHT_SECURE_ON_BAR),

      // set the arm to coast mode
      new SetArmCoast(m_climber),

      // retract the elevator
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_MIN_HEIGHT), // was Constants.ELEVATOR_FOR_ARM_CLEARANCE

      // set the arm to brake mode
      new SetArmBrake(m_climber),

      // clear from the bar
      new SetArmAngle(m_climber, Constants.ARM_CLIMB_BAR_OFFSET),

      new SetElevatorHeight(m_climber, Constants.ELEVATOR_HEIGHT_FOR_ARM_CLEARANCE,
           Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_ASCEND_SLOW, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_ASCEND_SLOW),

      // rotate the arm to leave the previous bar and get to the side of the next bar
      new SetArmAngle(m_climber, Constants.ARM_TO_THE_LEFT_ANGLE),

      // retract the elevator
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_MIN_HEIGHT),

      // rotates arm to grab on to the bar
      new SetArmAngle(m_climber, Constants.ARM_GRAB_THE_BAR),

      // extend the elevator a bit to come off the bar and let arm grab it
      new SetElevatorHeight(m_climber, Constants.ELEVATOR_HEIGHT_FOR_ARM_CLEARANCE,
           Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_ASCEND_SLOW, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_ASCEND_SLOW),
      
      new CommandFinished(true));
  }
}