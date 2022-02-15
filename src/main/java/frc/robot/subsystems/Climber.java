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
  CANSparkMax m_elevatorMotorLeader;
  CANSparkMax m_elevatorMotorFollower;
  TalonFX m_armMotorLeader;
  TalonFX m_armMotorFollower;
  RelativeEncoder m_winchEncoder;
  RelativeEncoder m_elevatorEncoder;
  SmartDashboard smartDashboard;
  SparkMaxPIDController m_elevatorPIDController, m_armPIDController;

  // PID coefficients for Elevator
  double m_kPElevator = 5e-5;
  double m_kIElevator = 1e-6;
  double m_kDElevator = 0; 
  double m_kIzElevator = 0; 
  double m_kFFElevator = 0.000156; 
  double m_kMaxOutputElevator = 1; 
  double m_kMinOutputElevator = -1;
  
  double m_elevatorMaxRPM = 5700;

  // Smart Motion Coefficients for Elevator
  double m_elevatorMaxVel = 2000; // rpm
  double m_elevatorMinVel = 0;
  double m_elevatorMaxAcc = 1500;

  double m_elevatorAllowedErr = 0;


  public Climber() {
    m_elevatorMotorLeader = new CANSparkMax(Constants.ELEVATOR_LEADER_CAN_ID, MotorType.kBrushless);
    m_elevatorMotorFollower = new CANSparkMax(Constants.ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
    m_armMotorLeader = new TalonFX(Constants.ARM_LEADER_CAN_ID);
    m_armMotorFollower = new TalonFX(Constants.ARM_FOLLOWER_CAN_ID);

    m_elevatorPIDController = m_elevatorMotorLeader.getPIDController();
    m_elevatorEncoder = m_elevatorMotorLeader.getEncoder();

    m_elevatorPIDController.setP(m_kPElevator);
    m_elevatorPIDController.setI(m_kIElevator);
    m_elevatorPIDController.setD(m_kDElevator);
    m_elevatorPIDController.setIZone(m_kIzElevator);
    m_elevatorPIDController.setFF(m_kFFElevator);
    m_elevatorPIDController.setOutputRange(m_kMinOutputElevator, m_kMaxOutputElevator);

    //set elevator PID smartMotion Coefficients
    int smartMotionSlot = 0;
    m_elevatorPIDController.setSmartMotionMaxVelocity(m_elevatorMaxVel, smartMotionSlot);
    m_elevatorPIDController.setSmartMotionMinOutputVelocity(m_elevatorMinVel, smartMotionSlot);
    m_elevatorPIDController.setSmartMotionMaxAccel(m_elevatorMaxAcc, smartMotionSlot);
    m_elevatorPIDController.setSmartMotionAllowedClosedLoopError(m_elevatorAllowedErr, smartMotionSlot);
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", m_kPElevator);
    SmartDashboard.putNumber("I Gain", m_kIElevator);
    SmartDashboard.putNumber("D Gain", m_kDElevator);
    SmartDashboard.putNumber("I Zone", m_kIzElevator
);
    SmartDashboard.putNumber("Feed Forward", m_kFFElevator);
    SmartDashboard.putNumber("Max Output", m_kMaxOutputElevator);
    SmartDashboard.putNumber("Min Output", m_kMinOutputElevator);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", m_elevatorMaxVel);
    SmartDashboard.putNumber("Min Velocity", m_elevatorMinVel);
    SmartDashboard.putNumber("Max Acceleration", m_elevatorMaxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", m_elevatorAllowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }
  public void periodic() {
    // This method will be called once per scheduler run
    checkElevatorPIDVal();
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

  private void checkElevatorPIDVal(){
   // read PID coefficients from SmartDashboard
   double p = SmartDashboard.getNumber("P Gain", 0);
   double i = SmartDashboard.getNumber("I Gain", 0);
   double d = SmartDashboard.getNumber("D Gain", 0);
   double iz = SmartDashboard.getNumber("I Zone", 0);
   double ff = SmartDashboard.getNumber("Feed Forward", 0);
   double max = SmartDashboard.getNumber("Max Output", 0);
   double min = SmartDashboard.getNumber("Min Output", 0);
   double maxV = SmartDashboard.getNumber("Max Velocity", 0);
   double minV = SmartDashboard.getNumber("Min Velocity", 0);
   double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
   double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

   // if PID coefficients on SmartDashboard have changed, write new values to controller
   if((p != m_kPElevator)) { m_elevatorPIDController.setP(p); m_kPElevator = p; }
   if((i != m_kIElevator)) { m_elevatorPIDController.setI(i); m_kIElevator = i; }
   if((d != m_kDElevator)) { m_elevatorPIDController.setD(d); m_kDElevator = d; }
   if((iz != m_kIzElevator)) { m_elevatorPIDController.setIZone(iz); m_kIzElevator = iz; }
   if((ff != m_kFFElevator)) { m_elevatorPIDController.setFF(ff); m_kFFElevator = ff; }
   if((max != m_kMaxOutputElevator) || (min != m_kMinOutputElevator)) { 
     m_elevatorPIDController.setOutputRange(min, max); 
     m_kMinOutputElevator = min; m_kMaxOutputElevator = max; 
   }
   if((maxV != m_elevatorMaxVel)) { m_elevatorPIDController.setSmartMotionMaxVelocity(maxV,0); m_elevatorMaxVel = maxV; }
   if((minV != m_elevatorMinVel)) { m_elevatorPIDController.setSmartMotionMinOutputVelocity(minV,0); m_elevatorMinVel = minV; }
   if((maxA != m_elevatorMaxAcc)) { m_elevatorPIDController.setSmartMotionMaxAccel(maxA,0); m_elevatorMaxAcc = maxA; }
   if((allE != m_elevatorAllowedErr)) { m_elevatorPIDController.setSmartMotionAllowedClosedLoopError(allE,0); m_elevatorAllowedErr = allE; }

   double setPoint, processVariable;
   boolean mode = SmartDashboard.getBoolean("Mode", false);
   if(mode) {
     setPoint = SmartDashboard.getNumber("Set Velocity", 0);
     m_elevatorPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
     processVariable = m_elevatorEncoder.getVelocity();
   } else {
     setPoint = SmartDashboard.getNumber("Set Position", 0);
     /**
      * As with other PID modes, Smart Motion is set by calling the
      * setReference method on an existing pid object and setting
      * the control type to kSmartMotion
      */
     m_elevatorPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
     processVariable = m_elevatorEncoder.getPosition();
   }
   
   SmartDashboard.putNumber("SetPoint", setPoint);
   SmartDashboard.putNumber("Process Variable", processVariable);
   SmartDashboard.putNumber("Output", m_elevatorMotorLeader.getAppliedOutput());
 }
}