// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  ClimberArm[] m_arm = new ClimberArm[2];
  Elevator[] m_elevator = new Elevator[2];

  double m_armMaxRPM = 5700;

  private double m_armGoal = 0.0;
  private double m_elevatorGoal = 0.0;

  // Smart Motion Coefficients for Arm
  double m_armMaxVel = 2000; // rpm
  double m_armMinVel = 0;
  double m_armMaxAcc = 100;

  double m_armAllowedErr = 0;

  boolean[] m_armTooFar = new boolean[] { false, false };

   double m_elevatorAllowedErr = 0;

  boolean[] m_elevatorTooFar = new boolean[] { false, false };

  double[] m_elevatorEncoderValue = new double[2];
  double[] m_armEncoderValue = new double[2];

  public Climber() {
    
    // Construct the arm trapezoid subsystems
    m_arm[0] = new ClimberArm(0, false);
    m_arm[1] = new ClimberArm(1, true);
    m_elevator[0] = new Elevator(0, false);
    m_elevator[1] = new Elevator(1, false);

    SmartDashboard.putNumber("arm/goal", m_armGoal);
    SmartDashboard.putNumber("elevator/goal", m_elevatorGoal);
    
  }

  public void periodic() {
    // All of the control should be done in the Trapeziodal subsystems
    // Check to see if the requested position has changed and then pass it to the Arm subsystems if needed
    double goal = SmartDashboard.getNumber("arm/goal", 0);
    if (goal != m_armGoal) {
      m_arm[0].setGoal(goal);
      m_arm[1].setGoal(goal);
      m_armGoal = goal;
    }
    goal = SmartDashboard.getNumber("elevator/goal", 0);
    if (goal != m_elevatorGoal) {
      m_elevator[0].setGoal(goal);
      m_elevator[1].setGoal(goal);
      m_elevatorGoal = goal;
    }
  }

  // sets the elevator to a certain height
  public void setElevatorHeight(double height) {

  }

  // rotates the arms to a certain angle
  public void setArmAngle(double degree) {

  }

  // returns the currrent height of the elevator
  public double getElevatorHeight() {
    return 0.0;
  }

  // returns the current angle of the arm
  public double getArmAngle() {
    return 0.0;
  }

  // Set idle mode of all motors
  public void setBrakeMode(boolean brake) {
    m_arm[0].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_arm[1].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevator[0].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevator[1].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }
}