package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Shooter.ShooterSpeeds;

import frc.robot.commands.ShooterCommand;
import frc.robot.RobotContainer;

public class ShooterCommandVacuum extends CommandBase {
    Shooter m_shooter;
    Intake m_intake;
    Vision m_vision;
    boolean m_upperHub;
    double m_distance;
    LigerTimer m_shootDelay = new LigerTimer(Constants.SHOOTER_MOTOR_WAIT_TIME);
    LigerTimer m_intakeDelay = new LigerTimer(Constants.SHOOTER_INTAKE_WAIT_TIME);
    LigerTimer m_shootBall1Time = new LigerTimer(Constants.SHOOT_BALL1_WAIT_TIME);
    LigerTimer m_shootBall2Time = new LigerTimer(Constants.SHOOT_BALL2_WAIT_TIME);
    LigerTimer m_visionTime = new LigerTimer(2.0);

    ShooterSpeeds m_shooterSpeeds;

    public ShooterCommandVacuum(Shooter shooter, Intake intake, Vision vision, boolean upperHub) {
        m_shooter = shooter;
        m_intake = intake;
        m_vision = vision;
        m_upperHub = upperHub;
    }

    public ShooterCommandVacuum(Shooter shooter, Intake intake, double distance, boolean upperHub) {
        m_shooter = shooter;
        m_intake = intake;
        m_distance = distance;
        m_upperHub = upperHub;
    }

    public ShooterCommandVacuum(Shooter shooter, Intake intake) {
        m_shooter = shooter;
        m_intake = intake;

        SmartDashboard.putNumber("shooter/Shooter Top Speed", 0);
        SmartDashboard.putNumber("shooter/Shooter Bottom Speed", 0);
        SmartDashboard.putNumber("shooter/Chute Speed", 0);
    }

    @Override
    public void initialize() {
        m_shootDelay.start();
        m_intakeDelay.start();
        // m_state = State.TURN_ON_CHUTE;
        // m_chute.start();

        double top = SmartDashboard.getNumber("shooter/Shooter Top Speed", 0);
        double bottom = SmartDashboard.getNumber("shooter/Shooter Bottom Speed", 0);
        m_shooter.setShooterRpms(top, bottom);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setChuteSpeed(0.0);
        m_shooter.setShooterRpms(0.0, 0.0);
        m_intake.run(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}