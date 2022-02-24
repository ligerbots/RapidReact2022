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

    LigerTimer m_shootDelay = new LigerTimer(Constants.SHOOTER_MOTOR_WAIT_TIME);
    LigerTimer m_intakeDelay = new LigerTimer(Constants.SHOOTER_INTAKE_WAIT_TIME);
    LigerTimer m_shotTime = new LigerTimer(Constants.SHOTS_WAIT_TIME);

    ShooterSpeeds m_shooterSpeeds;

    enum State {
        SPEED_UP_SHOOTER, WAIT_FOR_SHOOTER, TURN_ON_CHUTE_INTAKE, WAIT_FOR_SHOTS;
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
        double distance = m_vision.getDistance();
        m_shooterSpeeds = Shooter.calculateShooterSpeeds(distance);

        m_state = State.SPEED_UP_SHOOTER;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (m_state) {
            case SPEED_UP_SHOOTER:
                // turn on the two motors on the shooter, let the chute and intake wait for the
                // shots
                m_shooter.setShooterRpms(m_shooterSpeeds.top, m_shooterSpeeds.bottom);
                m_state = State.WAIT_FOR_SHOOTER;
                m_shootDelay.start();
                // TODO: comment why this is a fall through

            case WAIT_FOR_SHOOTER:
                if (m_shootDelay.hasElapsed())
                    m_state = State.TURN_ON_CHUTE_INTAKE;
                break;

            case TURN_ON_CHUTE_INTAKE:
                // turn on the chute once the shooter is ready
                m_shooter.setChuteSpeed(m_shooterSpeeds.chute);
                m_intake.run(Constants.INTAKE_SHOOTING_SPEED);
                m_state = State.WAIT_FOR_SHOTS;
                m_shotTime.start();
                break;
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
        return m_state == State.WAIT_FOR_SHOTS && m_shotTime.hasElapsed();
    }
}