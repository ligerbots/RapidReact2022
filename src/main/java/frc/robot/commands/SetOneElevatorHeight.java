// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetOneElevatorHeight extends TrapezoidProfileCommand {
  Climber m_climber;
  double m_height;
  int m_index;

  public SetOneElevatorHeight(Climber climber, double height, int index) {
    this(climber, height, index, Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_NORMAL, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_NORMAL);
  }

  public SetOneElevatorHeight(Climber climber, double height, int index, final double MAX_VEL_METER_PER_SEC, final double MAX_ACC_METER_PER_SEC) {
    super(
        new TrapezoidProfile(
            // Limit the max acceleration and velocity
            new TrapezoidProfile.Constraints(
                MAX_VEL_METER_PER_SEC,
                MAX_ACC_METER_PER_SEC),
            // End at desired position in meters; implicitly starts at 0
            new TrapezoidProfile.State(height, 0),
            // initial position state
            new TrapezoidProfile.State(climber.getElevatorHeight()[index], 0)),
        // Pipe the profile state to the drive
        setpointState -> climber.setOneElevatorHeight(index, setpointState));
      
        m_climber = climber;
        m_height = height;
  }
}
