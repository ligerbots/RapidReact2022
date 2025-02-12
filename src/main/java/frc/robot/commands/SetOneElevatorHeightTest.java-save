// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetOneElevatorHeightTest extends InstantCommand {
  Climber m_climber;
  public SetOneElevatorHeightTest(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(new SetOneElevatorHeight(m_climber,
     Units.inchesToMeters(SmartDashboard.getNumber("Constants/SetOneElevatorHeightTest", 0.0)), 
     (int) SmartDashboard.getNumber("Constants/OneElevatorIndex", 0.0)));
  }
}
