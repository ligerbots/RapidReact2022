package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
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

    double m_shooterStartTime;
    double m_chuteIntakeStartTime;

    double m_distance;

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
        m_distance = m_vision.getDistance();

        m_shooterSpeeds = Shooter.calculateShooterSpeeds(m_distance);

        m_state = State.SPEED_UP_SHOOTER;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (m_state) {
            case SPEED_UP_SHOOTER:
                // turn on the two motors on the shooter, let the chute and intake wait for the
                // shots
                m_shooter.shoot(m_shooterSpeeds.top, m_shooterSpeeds.bottom, 0.0);
                m_state = State.WAIT_FOR_SHOOTER;
                m_shooterStartTime = Robot.time();
            case WAIT_FOR_SHOOTER:
                if (Robot.time() - m_shooterStartTime > Constants.SHOOTER_MOTOR_WAIT_TIME)
                    m_state = State.TURN_ON_CHUTE_INTAKE;
                break;
            case TURN_ON_CHUTE_INTAKE:
                // turn on the chute once the shooter is ready
                m_shooter.shoot(m_shooterSpeeds.top, m_shooterSpeeds.bottom, m_shooterSpeeds.chute);
                m_intake.intakeCargo();
                m_chuteIntakeStartTime = Robot.time();
                m_state = State.WAIT_FOR_SHOTS;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0.0, 0.0, 0.0);
        m_intake.run(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_state == State.WAIT_FOR_SHOTS && Robot.time() - m_chuteIntakeStartTime > Constants.SHOTS_WAIT_TIME;
    }
}