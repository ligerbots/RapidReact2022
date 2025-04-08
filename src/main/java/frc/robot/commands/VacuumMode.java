package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterSpeeds;

public class VacuumMode extends Command{
    Shooter m_shooter;
    Intake m_intake;
    
    public VacuumMode(Shooter shooter, Intake intake){
        m_shooter = shooter;
        m_intake = intake;

    }

    @Override
    public void initialize() {
        //turn on shooter for vacuum mode using lowHub values
        ShooterSpeeds shooterSpeeds = Shooter.calculateShooterSpeeds(0.0, false);//lowerhub speeds for shooter
        m_shooter.setShooterRpms(shooterSpeeds.top, shooterSpeeds.bottom);

        //turn on chute for vacuum mode
        m_shooter.setChuteSpeed(shooterSpeeds.chute);

        //turn on intake for vacuum mode
        m_intake.run(Constants.INTAKE_SHOOTING_SPEED);
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
