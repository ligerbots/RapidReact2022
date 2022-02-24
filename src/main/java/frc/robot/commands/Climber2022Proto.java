// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class Climber2022Proto extends CommandBase {
  
  private Climber m_climber;
  private double m_armAngle;
  private double m_requestedPos;
  private final double m_tolerance = 3/360.0; // 3 degrees tolerance

  private double m_reference;

  /** Creates a new Climber2022Proto. */
  public Climber2022Proto(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Set to brake mode
    m_climber.setBrakeMode(true);
    
    m_armAngle = m_climber.getArmAngle();
    // in fraction from 0 to 1
    m_reference = m_armAngle;
    // put this initial valuer to SmartDashboard for reference
    SmartDashboard.putNumber("arm/Requested Arm Angle", (m_armAngle - m_reference) * 360.0);
      
  }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   SmartDashboard.putNumber("arm/Actual Arm Angle", (m_armAngle - m_reference)*360.0);
  //   // divide by 360 to get the fraction from 0 to 1 again
  //   // m_requestedPos is from 0-360 in degrees on the SMart Dashboard.
  //   // We need to save it as fraction from 0 to 1
  //   m_requestedPos = SmartDashboard.getNumber("arm/Requested Arm Angle", m_armAngle);
  //   m_requestedPos = m_requestedPos/360.0 + m_reference ;
    
	//   m_armAngle = m_climber.getArmAngle();

  //   if (m_requestedPos > m_armAngle + m_tolerance) {
	// 	// move up
	// 	if (m_armAngle > Constants.ARM_MAX_ANGLE) {
	// 		// We've gone high enough
  //     SmartDashboard.putString("Message", "High Enough, Hold the Shoulder");
	// 		m_climber.(Constants.SHOULDER_SPEED_HOLD);
	// 	}
	// 	else {
	// 		//move up
  //     SmartDashboard.putString("Message", "Moving Up");
	// 		m_climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_UP);
	// 	}
	// }
	// else if (m_requestedPos < m_armAngle - m_tolerance) {
	// 	// move down
	// 	if (m_armAngle < Constants.SHOULDER_MIN_HEIGHT) {
	// 		// We've gone down far enough
  //     SmartDashboard.putString("Message", "Low Enough, Hold the Shoulder");
	// 		m_climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_HOLD);
	// 	}
	// 	else {
  //     // move down
  //     SmartDashboard.putString("Message", "Moving Down");
	// 		m_climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_DOWN_FAST);
	// 	}
	// }
  // else{
  //   SmartDashboard.putString("Message", "Within the tolerance, Hold the Shoulder");
	// 		m_climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_HOLD);
  // }
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
		// Hold Position
		// m_climber.shoulder.setVoltage(Constants.SHOULDER_SPEED_HOLD);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   return Math.abs(m_requestedPos - m_climber.getShoulderPosition()) <= m_tolerance ||
	//  m_shoulderPos < Constants.SHOULDER_MIN_HEIGHT || m_shoulderPos > Constants.SHOULDER_MAX_HEIGHT;
  return false;
  }
}
