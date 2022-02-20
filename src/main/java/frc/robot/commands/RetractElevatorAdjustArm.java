// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
				new Command() {
					@Override
					public void initialize() {
						// put this in initialize since it only needs to be called once
						m_climber.setArmAngle(Constants.ARM_ADJUST_ANGLE);
					}

					// Called every time the scheduler runs while the command is scheduled.
					@Override
					public void execute() {

					}

					// Called once the command ends or is interrupted.
					@Override
					public void end(boolean interrupted) {
					}

					// Returns true when the command should end.
					@Override
					public boolean isFinished() {
						return Math.abs(
								m_climber.getArmAngle() - Constants.ARM_ADJUST_ANGLE) < Constants.ARM_ANGLE_TOLERANCE;
					}

					@Override
					public Set<Subsystem> getRequirements() {
						// TODO Auto-generated method stub
						return null;
					}
				},
				// command to retract the elevator
				new Command() {
					@Override
					public void initialize() {
						// put this in initialize since it only needs to be called once
						// + 20.0 to leave some spaces for the arm to rotate to the other side of the
						// bar later
						m_climber.setElevatorHeight(Constants.ELEVATOR_MIN_HEIGHT + 20.0);
					}

					// Called every time the scheduler runs while the command is scheduled.
					@Override
					public void execute() {

					}

					// Called once the command ends or is interrupted.
					@Override
					public void end(boolean interrupted) {
					}

					// Returns true when the command should end.
					@Override
					public boolean isFinished() {
						return Math.abs(m_climber.getElevatorHeight()
								- (Constants.ELEVATOR_MIN_HEIGHT + 20.0)) < Constants.ELEVATOR_HEIGHT_TOLERANCE;
					}

					@Override
					public Set<Subsystem> getRequirements() {
						// TODO Auto-generated method stub
						return null;
					}
				});
	}
}
