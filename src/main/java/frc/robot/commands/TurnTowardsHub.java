package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class TurnTowardsHub extends SequentialCommandGroup{
    DriveTrain m_driveTrain; 
    Vision m_vision;
    double m_toleranceDeg, m_stabilizeSecs;
    static final double kP = 0.01;
    class TurnTowardsHubLoop extends CommandBase {
        boolean m_withinTolerance;
        double m_lastOutsideToleranceTime;
        public TurnTowardsHubLoop() {
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            m_withinTolerance = false;
            m_lastOutsideToleranceTime = Timer.getFPGATimestamp();
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            double errorDeg = m_vision.getRobotAngle();
            m_withinTolerance = Math.abs(errorDeg) < m_toleranceDeg;
            if(!m_withinTolerance) m_lastOutsideToleranceTime = Timer.getFPGATimestamp();
            m_driveTrain.drive(0, -errorDeg * kP, false);
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            m_driveTrain.drive(0, 0, false);
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return Timer.getFPGATimestamp() - m_lastOutsideToleranceTime > m_stabilizeSecs;
        }
    }
    public TurnTowardsHub(double toleranceDeg, double stabilizeSecs, DriveTrain driveTrain, Vision vision){
        m_driveTrain = driveTrain;
        m_vision = vision;
        m_toleranceDeg = toleranceDeg;
        m_stabilizeSecs = stabilizeSecs;
        addCommands(
            new AwaitVisionResult(vision, Vision.VisionMode.HUBFINDER),
            new TurnTowardsHubLoop()
        );
    }
    public TurnTowardsHub(DriveTrain driveTrain, Vision vision){
        this(Constants.TURN_TOLERANCE_DEG, Constants.TURN_STABILIZE_SECS, driveTrain, vision);
    }
}
