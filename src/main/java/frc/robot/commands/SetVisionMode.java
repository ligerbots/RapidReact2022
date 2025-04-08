package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Vision;

public class SetVisionMode extends Command {
    Vision m_vision;
    Vision.VisionMode m_mode;
    
    public SetVisionMode(Vision vision, Vision.VisionMode mode){
        m_vision = vision;
        m_mode = mode;
    }

    @Override
    public void initialize() {
        m_vision.setMode(m_mode);
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
        return true;
    }
    
}
