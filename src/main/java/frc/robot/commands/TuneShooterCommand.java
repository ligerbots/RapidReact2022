package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneShooterCommand extends CommandBase{
    Shooter m_shooter;
    Intake m_intake;

    public TuneShooterCommand(Shooter shooter, Intake intake){
        m_shooter = shooter;
        m_intake = intake;

        SmartDashboard.putNumber("shooter/Shooter Top Speed", 0);
        SmartDashboard.putNumber("shooter/Shooter Bottom Speed", 0);
        SmartDashboard.putNumber("shooter/Chute Speed", 0);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double top = SmartDashboard.getNumber("shooter/Shooter Top Speed", 0);
        double bottom = SmartDashboard.getNumber("shooter/Shooter Bottom Speed", 0);
        double chute = SmartDashboard.getNumber("shooter/Chute Speed", 0);
        m_intake.run(Constants.INTAKE_SHOOTING_SPEED);
        m_shooter.shoot(top,bottom,chute);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.shoot(0,0,0);
        m_intake.run(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
