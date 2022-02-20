package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetClimber extends CommandBase {
  Climber m_climber;
  double m_rungHeight;
  double m_angle = Constants.CLIMBER_ANGLE;

  public SetClimber(Climber climber, double rungHeight) {
    m_climber = climber;
    m_rungHeight = rungHeight;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // only called at beginning when driver clicks button, only needs to execute
    // once

    m_climber.setArmAngle(m_angle);
    m_climber.setElevatorHeight(m_rungHeight);
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
    // is finished if the distance between the getElevatorHeight()

    return (Math.abs(m_rungHeight - m_climber.getElevatorHeight()) < Constants.ARM_ANGLE_TOLERANCE);
  }
}
