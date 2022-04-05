// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorHeight extends TrapezoidProfileCommand {
  boolean m_goingDown;
  Climber m_climber;
  double m_height;

  // Did we hit the limit switch?
  boolean[] m_hitLimitSwitch;
  boolean[] m_limitSwitchAlreadyPressed;

  boolean[] m_ressetEncoder;

  public SetElevatorHeight(Climber climber, double height) {
    this(climber, height, Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_NORMAL, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_NORMAL);
  }


  public SetElevatorHeight(Climber climber, double height, final double MAX_VEL_METER_PER_SEC, final double MAX_ACC_METER_PER_SEC) {
    super(
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(
                MAX_VEL_METER_PER_SEC,
                MAX_ACC_METER_PER_SEC),
            // End at desired position in meters; implicitly starts at 0
            new TrapezoidProfile.State(height, 0),
            // initial position state
            // TODO: it may not work when two elevators are executed separately at different hight
            new TrapezoidProfile.State(climber.getElevatorHeight()[0], 0)),
        // Pipe the profile state to the drive
        setpointState -> climber.setElevatorHeight(setpointState));
      
        m_climber = climber;
        m_height = height;
  }
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("SetElevatorHeight finished", super.isFinished());
    return super.isFinished();
  }


  //   // Called when the command is initially scheduled.
  //   @Override
  //   public void initialize() {
  //     super.initialize();
  //     // Initialize each height
  //     m_hitLimitSwitch[0] = false;
  //     m_hitLimitSwitch[1] = false;
  //     m_ressetEncoder[0] = false;
  //     m_ressetEncoder[1] = false;
  //     m_goingDown = m_climber.getElevatorHeight()[0] > m_height;
  //   }

  //   @Override
  //   public void execute() {
  //     super.execute();
  //     // If we're going down, we have to check things here and potentially change
  //     // the
  //     // requested elevator height.
  //     if (m_goingDown) {
  //       // We're going to need to check each elevator independently, so we need a loop
  //       for (int i = 0; i <= 1; i++) {  
  //         // Make sure that the switch is pressed twice in a row.
  //         boolean tempPressedCheck = m_climber.m_limitSwitch[i].isPressed() &&
  //             m_climber.getElevatorHeight()[i] < Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT;
  //         m_hitLimitSwitch[i] = tempPressedCheck && m_limitSwitchAlreadyPressed[i];
  //         m_limitSwitchAlreadyPressed[i] = tempPressedCheck;
  
  //         // If we've hit the limit switch twice, we need to set the encoder value to limit switch height
  //         // and then raise elevator to ELEVATOR_MIN_HEIGHT.
  //         if (m_hitLimitSwitch[i] && m_limitSwitchAlreadyPressed[i] && !m_ressetEncoder[i]) {
  //           m_climber.m_elevatorMotor[i].getEncoder().setPosition(Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT);
  //           m_climber.setOneElevatorHeight(i, new TrapezoidProfile.State(Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT, 0.0));
  //           m_ressetEncoder[i] = true;
  //         }
  //       }
  //     }
  // }
  
  // @Override
  // public boolean isFinished() {
  //   boolean finished = super.isFinished();
  //   if(m_goingDown){
  //     return finished || (m_hitLimitSwitch[0] && m_hitLimitSwitch[1]);
  //   }else{
  //     return finished;
  //   }
  // }
}
