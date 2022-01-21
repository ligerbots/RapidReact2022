package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
  */

  Intake m_intake;
  Climber m_climber;
  double m_speed;
  public IntakeCommand(Intake intake, double speed) {
    m_intake = intake;
    //this.climber = climber;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //climber.shoulder.setIdleMode(IdleMode.kCoast);
    m_intake.run(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_intake.run(0);
      //climber.shoulder.setIdleMode(IdleMode.kBrake);
  }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}