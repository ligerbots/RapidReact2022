package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
enum State{
  RAISE_ROBOT, GRAB_BAR, LOWER_ROBOT;
}
public class raiseToBar extends CommandBase {
    Climber m_climber;
    State m_state;
    double m_rungAngle = Constants.RUNG_ANGLE;//desired arm angle to hook onto rung
    double m_setRetractElevator = Constants.ELEVATOR_RETRACT_HEIGHT;//height it lowers elevator to in order to raise robot
    double m_postHeight = Constants.POSTGRAB_ELEVATOR_HEIGHT;//
    public raiseToBar(Climber climber) {
        m_climber = climber;
        addRequirements(climber);
      }
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_state = State.RAISE_ROBOT;//initialize to raise robot
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(m_climber.getArmAngle()>Constants.CLIMBER_ANGLE){//if arm angle is greater then what it should be for climbing, then set
        m_climber.rotateArm(Constants.CLIMBER_ANGLE);
      }
      switch(m_state){
        case RAISE_ROBOT:
            m_climber.setElevatorHeight(m_setRetractElevator);//retracts elevator to desired height, is a constant
            //if distance between current elevator height and desired retracted elevator height is less then tolerance, continue
            if(Math.abs(m_climber.getElevatorHeight()-m_setRetractElevator)<Constants.ELEVATORHEIGHT_TOLERANCE){
              m_state = State.GRAB_BAR; //grabs bar
            }
            
            break;
        case GRAB_BAR:
            m_climber.rotateArm(m_rungAngle);//is a constant
            //if distance between current arm angle and desired angle to hook onto rung  is less then tolerance, continue
            if(Math.abs(m_climber.getArmAngle()-m_rungAngle)<Constants.ARMANGLE_TOLERANCE){
              m_state = State.LOWER_ROBOT; //lowers robot
            }
            
            break;
        case LOWER_ROBOT:
            m_climber.setElevatorHeight(Constants.POSTGRAB_ELEVATOR_HEIGHT);
            //if distance between current elevator height and desired height to extend elevator is less then tolerance, continue
            
            break;
      }

  }
  

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_climber.getElevatorHeight()-m_postHeight)<Constants.ELEVATORHEIGHT_TOLERANCE);
  }

  
  

  }