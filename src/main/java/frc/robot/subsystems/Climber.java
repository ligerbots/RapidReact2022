// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  double m_maxAccell, m_maxVel,m_allowedErr,m_minVel;
  CANSparkMax m_elevatorMotorLeader;
  CANSparkMax m_elevatorMotorFollower;
  TalonFX m_armMotorLeader;
  TalonFX m_armMotorFollower;
  RelativeEncoder m_winchEncoder;
  RelativeEncoder m_elevatorEncoder;
  SmartDashboard smartDashboard;
  SparkMaxPIDController m_elevatorPIDController, m_armPIDController;

  public Climber() {
    m_elevatorMotorLeader = new CANSparkMax(Constants.ELEVATOR_LEADER_CAN_ID, MotorType.kBrushless);
    m_elevatorMotorFollower = new CANSparkMax(Constants.ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
    m_armMotorLeader = new TalonFX(Constants.ARM_LEADER_CAN_ID);
    m_armMotorFollower = new TalonFX(Constants.ARM_FOLLOWER_CAN_ID);

    m_elevatorPIDController = m_elevatorMotorLeader.getPIDController();
    m_elevatorEncoder = m_elevatorMotorLeader.getEncoder();

    m_elevatorPIDController.setP(Constants.kPElevator);
    m_elevatorPIDController.setI(Constants.kIElevator);
    m_elevatorPIDController.setD(Constants.kDElevator);
    m_elevatorPIDController.setIZone(Constants.kIzElevator);
    m_elevatorPIDController.setFF(Constants.kFFElevator);
    m_elevatorPIDController.setOutputRange(Constants.kMinOutputElevator, Constants.kMaxOutputElevator);

    //set elevator PID smartMotion Coefficients
    m_maxAccell = Constants.ElevatorMaxRPM;
    m_maxVel = Constants.ElevatorMaxVel;

    int smartMotionSlot = 0;
    m_elevatorPIDController.setSmartMotionMaxVelocity(m_maxVel, smartMotionSlot);
    m_elevatorPIDController.setSmartMotionMinOutputVelocity(m_minVel, smartMotionSlot);
    m_elevatorPIDController.setSmartMotionMaxAccel(m_maxAccell, smartMotionSlot);
    m_elevatorPIDController.setSmartMotionAllowedClosedLoopError(m_allowedErr, smartMotionSlot);
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", Constants.kPElevator);
    SmartDashboard.putNumber("I Gain", Constants.kIElevator);
    SmartDashboard.putNumber("D Gain", Constants.kDElevator);
    SmartDashboard.putNumber("I Zone", Constants.kIzElevator);
    SmartDashboard.putNumber("Feed Forward", Constants.kFFElevator);
    SmartDashboard.putNumber("Max Output", Constants.kMaxOutputElevator);
    SmartDashboard.putNumber("Min Output", Constants.kMinOutputElevator);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", m_maxVel);
    SmartDashboard.putNumber("Min Velocity", m_minVel);
    SmartDashboard.putNumber("Max Acceleration", m_maxAccell);
    SmartDashboard.putNumber("Allowed Closed Loop Error", m_allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  // sets the elevator to a certain height
  public void setElevatorHeight(double height) {

  }
  
  // rotates the arms a certain degree
  public void rotateArm(double degree){

  }

  // returns the currrent height of the elevator
  public void getElevatorHeight(){

  }

  //returns the current angle of the arm
  public void getArmAngle(){

  }


}
