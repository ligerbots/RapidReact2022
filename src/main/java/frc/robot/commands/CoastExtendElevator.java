// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;

public class CoastExtendElevator extends CommandBase {

  private Climber m_climber;
  private int iterations = 0;
  private double[] m_elevatorHeight = new double[2];
  private double[] m_lastElevatorHeight = new double[2];


  /** Creates a new CoastExtendElevator. */
  public CoastExtendElevator(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // set the Elevators to Coast mode
    m_climber.setElevatorCoast(true);
    // Set elevator voltage to low level to get it moving and stop the PID
    m_climber.setElevatorVoltage(1.0);

    // Save the latest elevator height
    m_lastElevatorHeight = m_climber.getElevatorHeight();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Read elevator heights for later check
    m_elevatorHeight = m_climber.getElevatorHeight();

    // increment the counter
    iterations++;

    // Let it keep the voltage briefly
    if (iterations >=2){
      // Stop the elevator motor, but since it's in coast, the Constant Force spring sholuld let the elevator go all the way up.
      m_climber.setElevatorVoltage(0.0);
      // Read the elevator heights again. Since it's been two iterations, we can check if the elevator heights have changed.
      // This will be check in isFinished
      m_elevatorHeight = m_climber.getElevatorHeight();
    }
    iterations++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        // set the Elevators back to Brake mode for the subsequent commands
        m_climber.setElevatorCoast(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Let it run for at least a few iterations so it should not be stuck and then check to see
    // if it is fully extended by the springs. This will mean that we get virtually the same
    // encoder reading twice in a row.
    return (iterations > 5 && Math.abs(m_elevatorHeight[0] - m_lastElevatorHeight[0]) < 0.05 && 
      Math.abs(m_elevatorHeight[1] -  m_lastElevatorHeight[1]) < 0.05);
  }
}
