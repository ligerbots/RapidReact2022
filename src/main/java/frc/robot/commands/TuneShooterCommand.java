package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TuneShooterCommand extends CommandBase{
    Shooter m_shooter;
    Intake m_intake;
    LigerTimer m_shootDelay = new LigerTimer(Constants.SHOOTER_MOTOR_WAIT_TIME);
    LigerTimer m_intakeDelay = new LigerTimer(Constants.SHOOTER_MOTOR_WAIT_TIME + Constants.SHOOTER_INTAKE_WAIT_TIME);

    public TuneShooterCommand(Shooter shooter, Intake intake){
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
    
        double top = SmartDashboard.getNumber("shooter/Shooter Top Speed", 0);
        double bottom = SmartDashboard.getNumber("shooter/Shooter Bottom Speed", 0);
        m_shooter.setShooterRpms(top, bottom);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_shootDelay.hasElapsed()){
            double chute = SmartDashboard.getNumber("shooter/Chute Speed", 0);
            m_shooter.setChuteSpeed(chute);
        }
        
        if (m_intakeDelay.hasElapsed()) {
            m_intake.run(Constants.INTAKE_SHOOTING_SPEED);
        }
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
