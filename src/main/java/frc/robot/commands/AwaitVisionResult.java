package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionMode;

public class AwaitVisionResult extends CommandBase {
    private Vision m_vision;
    private VisionMode m_mode;
    public AwaitVisionResult(Vision vision, VisionMode mode) {
        m_vision = vision;
        m_mode = mode;
    }
    @Override
    public void initialize() {
        m_vision.setMode(m_mode);
    }
    @Override
    public boolean isFinished() {
        return m_vision.getStatus() && m_vision.getDistance() > 0;
    }
    @Override
    public void execute() {
    }
    @Override
    public void end(boolean interrupted) {
    }
}