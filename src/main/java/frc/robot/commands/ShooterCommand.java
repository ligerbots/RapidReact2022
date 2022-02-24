package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase{
    Shooter m_shooter;
    Intake m_intake;
    Vision m_vision;

    public ShooterCommand(Shooter shooter, Intake intake, Vision vision) {
        m_shooter = shooter;
        m_intake = intake;
        m_vision = vision;
    }

    @Override
    public void initialize() {
        System.out.println("*sound of shooter shooting*"); // debugging
    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_vision.setMode(Vision.DEFAULT_MODE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

}
