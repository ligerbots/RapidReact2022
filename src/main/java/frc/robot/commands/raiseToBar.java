package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.ClimbToNextBar.STAGE;
import frc.robot.subsystems.Climber;

public class RaiseToBar extends CommandBase {
	enum State {
		RAISE_ROBOT, WAIT_RAISE_ROBOT, WAIT_GRAB_BAR, GRAB_BAR, WAIT_LOWER_ROBOT, LOWER_ROBOT, FINISHED;
	}

	Climber m_climber;
	State m_state;
	double m_rungAngle = Constants.RUNG_ANGLE;// desired arm angle to hook onto rung
	double m_setRetractElevator = Constants.ELEVATOR_RETRACT_HEIGHT;// height it lowers elevator to in order to raise
																	// robot
	double m_postHeight = Constants.POSTGRAB_ELEVATOR_HEIGHT;//

	public RaiseToBar(Climber climber) {
		m_climber = climber;
		addRequirements(climber);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_state = State.RAISE_ROBOT;// initialize to raise robot
		if (m_climber.getArmAngle() > Constants.CLIMBER_ANGLE) {// if arm angle is greater then what it should be for
																// climbing, then set
			m_climber.setArmAngle(Constants.CLIMBER_ANGLE);
		}

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		switch (m_state) {
			case RAISE_ROBOT:														
					// retracts elevator to desired height, is a constant
					m_climber.setElevatorHeight(m_setRetractElevator);
					m_state = State.WAIT_RAISE_ROBOT;

			case WAIT_RAISE_ROBOT:
					if (Math.abs(m_climber.getElevatorHeight() - m_setRetractElevator) < Constants.ELEVATOR_HEIGHT_TOLERANCE) 
						m_state = State.GRAB_BAR; // grabs bar
					break;	
			case GRAB_BAR:
					m_climber.setArmAngle(m_rungAngle);// is a constant
					m_state = State.WAIT_GRAB_BAR;
			case WAIT_GRAB_BAR:
				if (Math.abs(m_climber.getArmAngle() - m_rungAngle) < Constants.ARM_ANGLE_TOLERANCE) 
					m_state = State.LOWER_ROBOT; // lowers robot
				break;
			case LOWER_ROBOT:
					m_climber.setElevatorHeight(Constants.POSTGRAB_ELEVATOR_HEIGHT);
					m_state = State.WAIT_LOWER_ROBOT;
			case WAIT_LOWER_ROBOT:
					if(Math.abs(m_climber.getElevatorHeight() - m_postHeight) < Constants.ELEVATOR_HEIGHT_TOLERANCE)
						m_state = State.FINISHED;
				break;
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_state == State.FINISHED;
	}

}