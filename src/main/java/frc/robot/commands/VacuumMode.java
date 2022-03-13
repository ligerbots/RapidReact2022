package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterSpeeds;

public class VacuumMode extends CommandBase{
    Shooter m_shooter;
    Intake m_intake;
    double m_distance;
    ShooterSpeeds m_shooterSpeeds;

    public VacuumMode(Shooter shooter, Intake intake){
        m_shooter = shooter;
        m_intake = intake;

        SmartDashboard.putNumber("shooter/Shooter Top Speed", 0);
        SmartDashboard.putNumber("shooter/Shooter Bottom Speed", 0);
        SmartDashboard.putNumber("shooter/Chute Speed", 0);
    }

    @Override
    public void initialize() {
        /*
        double top = SmartDashboard.getNumber("shooter/Shooter Top Speed", 0);
        double bottom = SmartDashboard.getNumber("shooter/Shooter Bottom Speed", 0);
        m_shooter.setShooterRpms(top, bottom);*/

        m_shooterSpeeds = Shooter.calculateShooterSpeeds(m_distance, false);//lowerhub speeds for shooter
        m_shooter.setShooterRpms(m_shooterSpeeds.top, m_shooterSpeeds.bottom);

        //turn on chute for vacuum mode
        double chute = SmartDashboard.getNumber("shooter/Chute Speed", 0);
        m_shooter.setChuteSpeed(chute);

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
