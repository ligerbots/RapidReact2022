package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.ShooterSpeeds;

public class ShooterCommand extends CommandBase {
    /**
     * Creates a new ShooterCommand.
     */
    Shooter m_shooter;
    Intake m_intake;
    Vision m_vision;

    double m_distance;

    LigerTimer m_shootDelay = new LigerTimer(Constants.SHOOTER_MOTOR_WAIT_TIME);
    LigerTimer m_intakeDelay = new LigerTimer(Constants.SHOOTER_INTAKE_WAIT_TIME);
    LigerTimer m_shotTime = new LigerTimer(Constants.SHOTS_WAIT_TIME);
    LigerTimer m_intakeTime = new LigerTimer(Constants.INTAKE_WAIT_TIME);

    ShooterSpeeds m_shooterSpeeds;

    enum State {
        FINDING_VISION_TARGET, SPEED_UP_SHOOTER, WAIT_FOR_SHOOTER, TURN_ON_CHUTE, TURN_ON_INTAKE, WAIT_FOR_SHOTS, WAIT_FOR_INTAKE;
    }

    State m_state;

    public ShooterCommand(Shooter shooter, Intake intake, Vision vision) {
        m_shooter = shooter;
        m_intake = intake;
        m_vision = vision;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_state = State.SPEED_UP_SHOOTER;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (m_state) {
            case FINDING_VISION_TARGET:
                m_distance = m_vision.getDistance();
                if(m_distance != 0.0)
                    m_state = State.SPEED_UP_SHOOTER;
                break;
            case SPEED_UP_SHOOTER:
                m_shooterSpeeds = Shooter.calculateShooterSpeeds(m_distance);
                // turn on the two motors on the shooter, let the chute and intake wait for the
                // shots
                m_shooter.setShooterRpms(m_shooterSpeeds.top, m_shooterSpeeds.bottom);
                m_state = State.WAIT_FOR_SHOOTER;
                m_shootDelay.start();
                // able to go straight to the next state since this only needs to be called once
                // and can start checking directly

            case WAIT_FOR_SHOOTER:
                if (m_shootDelay.hasElapsed())
                    m_state = State.TURN_ON_CHUTE;
                break;

            case TURN_ON_CHUTE:
                // turn on the chute once the shooter is ready
                m_shooter.setChuteSpeed(m_shooterSpeeds.chute);
                m_state = State.WAIT_FOR_SHOTS;
                m_shotTime.start();
                // same logics, able to go to the next state directly
                
            case WAIT_FOR_SHOTS:
                if(m_shotTime.hasElapsed())
                    m_state = State.TURN_ON_INTAKE;
                break;
            
            case TURN_ON_INTAKE:
                m_intake.run(Constants.INTAKE_SHOOTING_SPEED);
                m_intakeTime.start();
                m_state = State.WAIT_FOR_INTAKE;
                break;

            // State.WAIT_FOR_INTAKE checked in isFinished method
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setShooterRpms(0.0, 0.0);
        m_shooter.setChuteSpeed(0.0);
        m_intake.run(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_state == State.WAIT_FOR_INTAKE && m_intakeTime.hasElapsed();
    }
}