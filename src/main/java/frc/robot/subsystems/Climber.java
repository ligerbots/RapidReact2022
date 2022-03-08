// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public ClimberArm[] m_arm = new ClimberArm[2];
  public ElevatorAscend[] m_elevatorAscend = new ElevatorAscend[2];

  
  public ElevatorDescend[] m_elevatorDescend = new ElevatorDescend[2];

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
    m_elevatorAscend[0] = new ElevatorAscend(0, false, this);
    m_elevatorAscend[1] = new ElevatorAscend(1, true, this);
    m_elevatorDescend[0] = new ElevatorDescend(0, false, this);
    m_elevatorDescend[1] = new ElevatorDescend(1, true, this);

    SmartDashboard.putNumber("arm/goal", m_armGoal);
    SmartDashboard.putNumber("elevator/goal", m_elevatorGoal);
  }

  public void periodic() {
    // All of the control should be done in the Trapeziodal subsystems
    // Check to see if the requested position has changed and then pass it to the Arm subsystems if needed
    // double goal = SmartDashboard.getNumber("arm/goal", Math.toDegrees(Constants.ARM_OFFSET_RAD));
    // if (goal != m_armGoal) {
    //   double goalUnits = Units.degreesToRadians(goal);
    //   m_arm[0].setGoal(goalUnits);
    //   m_arm[1].setGoal(goalUnits);
    //   m_armGoal = goal;
    // }

    // goal = SmartDashboard.getNumber("elevator/goal", 0);
    // if (goal != m_elevatorGoal) {
    //   double goalUnits = Units.inchesToMeters(goal);
    //   //m_elevator[0].setGoal(goalUnits);
    //   m_elevator[1].setGoal(goalUnits); //* (20.1/18.45) factor for lowest point
    //   m_elevatorGoal = goal;
    // }
  }

  // sets the elevator to a certain height
  public void setElevatorHeight(double height) {
    double curHeight = getElevatorHeight()[0];//both motors are similar, getting current height  
    if(height>curHeight){
        m_elevatorAscend[0].resetElevatorPos();
        m_elevatorAscend[1].resetElevatorPos();
        m_elevatorAscend[0].elevatorAscending();//either ascending or descending
        m_elevatorAscend[1].elevatorAscending();
        m_elevatorAscend[0].setGoal(height);
        m_elevatorAscend[1].setGoal(height);
      }else{
        m_elevatorDescend[0].resetElevatorPos();
        m_elevatorDescend[1].resetElevatorPos();
        m_elevatorDescend[0].elevatorDescending();
        m_elevatorDescend[1].elevatorDescending();
        m_elevatorDescend[0].setGoal(height);
        m_elevatorDescend[1].setGoal(height);
      }
      
  }

  // rotates the arms to a certain angle
  public void setArmAngle(double degree) {
      m_arm[0].setGoal(degree);
      m_arm[1].setGoal(degree);

  }

  // returns the currrent height of the elevator
  public double[] getElevatorHeight() {
    return new double[] {m_elevatorAscend[0].getEncoder().getPosition(), m_elevatorAscend[1].getEncoder().getPosition()};//does not matter bc both elevatorAscend and elevatorDescend share same motor
  }

  // returns the current angle of the arm
  public double[] getArmAngle() {
    return new double[] {m_arm[0].getEncoder().getPosition(), m_arm[1].getEncoder().getPosition()};
  }

  // Set idle mode of all motors
  public void setBrakeMode(boolean brake) {
    m_arm[0].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_arm[1].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevatorAscend[0].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);//does not matter bc both elevatorAscend and elevatorDescend share same motor
    m_elevatorAscend[1].getMotor().setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  public void setArmCoastMode(){
    m_arm[0].idleMotor();
    m_arm[1].idleMotor();
  }

  public void unsetArmCoastMode(){
    m_arm[0].unIdleMotor();
    m_arm[1].unIdleMotor();
  }
}