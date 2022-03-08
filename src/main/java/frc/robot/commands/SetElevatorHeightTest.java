// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetElevatorHeightTest extends CommandBase {
  /** Creates a new SetElevatorHeight. */
  Climber m_climber;
  double m_height;
  String m_key;
  public SetElevatorHeightTest(Climber climber, String key) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_key = key;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_height = Units.inchesToMeters(SmartDashboard.getNumber(m_key, 0.0));
    m_climber.setElevatorHeight(m_height);
    SmartDashboard.putString("Message", "SetElevatorHeightTest executed!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double[] arr = m_climber.getElevatorHeight();
    return Math.abs(arr[0] - m_height) < Constants.ELEVATOR_HEIGHT_TOLERANCE
    || Math.abs(arr[1] - m_height) < Constants.ELEVATOR_HEIGHT_TOLERANCE;
  }
}