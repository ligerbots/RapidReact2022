package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterTable;
import frc.robot.subsystems.Vision;

import java.util.TreeMap;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase{
    Shooter m_shooter;
    Intake m_intake;
    Vision m_vision;
    TreeMap <Double, ShooterTable> m_speedTable = new TreeMap <Double, ShooterTable> (){}; //set up lookup table for shooter speeds

    public ShooterCommand(Shooter shooter, Intake intake, Vision vision){
        m_shooter = shooter;
        m_intake = intake;
        m_vision = vision;
    }

    @Override
    public void initialize() {
        m_speedTable.put(0.0, new ShooterTable(1.0, 1.0, 1.0));
        m_speedTable.put(20.0, new ShooterTable(2.0, 2.0, 2.0));
        m_speedTable.put(40.0, new ShooterTable(3.0, 3.0, 3.0));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
