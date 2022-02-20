package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneShooter extends CommandBase{
    Shooter m_shooter;
    Double top, bottom, chute;
    SmartDashboard smartDashboard;
    public TuneShooter(Shooter shooter){
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Shooter Top Speed", 0);
        SmartDashboard.putNumber("Shooter Bottom Speed", 0);
        SmartDashboard.putNumber("Chute Speed", 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        top = SmartDashboard.getNumber("Shooter Top Speed", 0);
        bottom = SmartDashboard.getNumber("Shooter Bottom Speed", 0);
        chute = SmartDashboard.getNumber("Chute Speed", 0);
        m_shooter.shoot(top,bottom,chute);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_shooter.shoot(0,0,0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
