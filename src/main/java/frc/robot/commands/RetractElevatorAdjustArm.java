// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractElevatorAdjustArm extends ParallelCommandGroup {
	/** Creates a new RetractElevatorAdjustArm. */
	public Climber m_climber;

	public RetractElevatorAdjustArm(Climber climber) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		m_climber = climber;
		addCommands(
				// command to rotate the arm
				new SetArmAngle(m_climber, Constants.ARM_ADJUST_ANGLE),
				// command to retract the elevator, while leave some spaces for arms to get to the other side of the bar later
				new SetElevatorHeight(m_climber, Constants.ELEVATOR_HEIGHT_FOR_ARM_CLEARANCE));
	}
}
