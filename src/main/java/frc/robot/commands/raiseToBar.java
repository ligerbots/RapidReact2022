package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
public class raiseToBar extends CommandBase {
    Climber m_climber;
    double m_rungHeight;
    double m_angle = Constants.CLIMBER_ANGLE;
    public raiseToBar(Climber climber, double rungHeight) {
        m_climber = climber;
        m_rungHeight = rungHeight;
        addRequirements(climber);
      }
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      String stage = "raiseRobot";

      if(stage=="raiseRobot"){
        raise the thing because yefklw;'';
      }
      else{
          break;
      }

    }
    System.out.println(monthString);
}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
//1. raise elevator. make sure arms are out of the way
//2. 
  
  

  }