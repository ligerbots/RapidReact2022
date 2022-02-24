// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  CANSparkMax m_elevatorMotorLeader;
  CANSparkMax m_elevatorMotorFollower;
  CANSparkMax m_armMotorLeader;
  CANSparkMax m_armMotorFollower;
  DutyCycleEncoder throughBoreEncoder;
  RelativeEncoder m_winchEncoder;
  RelativeEncoder m_armEncoder;
  SmartDashboard smartDashboard;
  SparkMaxPIDController m_elevatorPIDController, m_armPIDController;

  // PID coefficients for Arm
  double m_kPArm = 5e-5;
  double m_kIArm = 1e-6;
  double m_kDArm = 0; 
  double m_kIzArm = 0; 
  double m_kFFArm = 0.000156; 
  double m_kMaxOutputArm = 1; 
  double m_kMinOutputArm = -1;
  
  double m_armMaxRPM = 5700;

  // Smart Motion Coefficients for Elevator
  double m_armMaxVel = 2000; // rpm
  double m_armMinVel = 0;
  double m_armMaxAcc = 100;

  double m_armAllowedErr = 0;

  boolean m_armTooFar = false;


  public Climber() {
    m_armMotorLeader = new CANSparkMax(Constants.ARM_LEADER_CAN_ID, MotorType.kBrushless);
    m_armMotorFollower = new CANSparkMax(Constants.ARM_FOLLOWER_CAN_ID, MotorType.kBrushless);
    m_elevatorMotorLeader = new CANSparkMax(Constants.ELEVATOR_LEADER_CAN_ID,MotorType.kBrushless);
    m_elevatorMotorFollower = new CANSparkMax(Constants.ELEVATOR_FOLLOWER_CAN_ID,MotorType.kBrushless);

    // This will reset the encoder value to 0
    m_armMotorLeader.restoreFactoryDefaults();
    m_armPIDController = m_armMotorLeader.getPIDController();
    m_armEncoder = m_armMotorLeader.getEncoder();
    // gear Rtio for Arm is 25::1 Max Planetary and 60::16 chain reduction
    m_armEncoder.setPositionConversionFactor((1.0/(25.0*60.0/16.0))*360.0);
    System.out.println("Get Position Conversion Factor");
    m_armPIDController.setP(m_kPArm);
    m_armPIDController.setI(m_kIArm);
    m_armPIDController.setD(m_kDArm);
    m_armPIDController.setIZone(m_kIzArm);
    m_armPIDController.setFF(m_kFFArm);
    m_armPIDController.setOutputRange(m_kMinOutputArm, m_kMaxOutputArm);

    //set elevator PID smartMotion Coefficients
    int smartMotionSlot = 0;
    m_armPIDController.setSmartMotionMaxVelocity(m_armMaxVel, smartMotionSlot);
    m_armPIDController.setSmartMotionMinOutputVelocity(m_armMinVel, smartMotionSlot);
    m_armPIDController.setSmartMotionMaxAccel(m_armMaxAcc, smartMotionSlot);
    m_armPIDController.setSmartMotionAllowedClosedLoopError(m_armAllowedErr, smartMotionSlot);
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", m_kPArm);
    SmartDashboard.putNumber("I Gain", m_kIArm);
    SmartDashboard.putNumber("D Gain", m_kDArm);
    SmartDashboard.putNumber("I Zone", m_kIzArm
);
    SmartDashboard.putNumber("Feed Forward", m_kFFArm);
    SmartDashboard.putNumber("Max Output", m_kMaxOutputArm);
    SmartDashboard.putNumber("Min Output", m_kMinOutputArm);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", m_armMaxVel);
    SmartDashboard.putNumber("Min Velocity", m_armMinVel);
    SmartDashboard.putNumber("Max Acceleration", m_armMaxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", m_armAllowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }
  public void periodic() {
    // This method will be called once per scheduler run
    checkArmPIDVal();

    // If we go too far in either direction, shut it down
    if (m_armEncoder.getPosition() > Constants.ARM_MAX_ANGLE ||
    m_armEncoder.getPosition() < Constants.ARM_MIN_ANGLE) {
      m_armMotorLeader.stopMotor();
      m_armTooFar = true;
    } else {
      m_armTooFar = false;
    }
    SmartDashboard.putBoolean("arm/Too Far", m_armTooFar);
  }
  
  // sets the elevator to a certain height
  public void setElevatorHeight(double height) {

  }
  
  // rotates the arms to a certain angle
  public void setArmAngle(double degree){

  }

  // returns the currrent height of the elevator
  public double getElevatorHeight(){
    return 0.0;
  }

  //returns the current angle of the arm
  public double getArmAngle(){
    return 0.0;
  }

  // Set idle mode of all motors
  public void setBrakeMode(boolean brake){
    m_armMotorLeader.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_armMotorFollower.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevatorMotorLeader.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevatorMotorFollower.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  private void checkArmPIDVal(){
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
   if((p != m_kPArm)) { m_armPIDController.setP(p); m_kPArm = p; }
   if((i != m_kIArm)) { m_armPIDController.setI(i); m_kIArm = i; }
   if((d != m_kDArm)) { m_armPIDController.setD(d); m_kDArm = d; }
   if((iz != m_kIzArm)) { m_armPIDController.setIZone(iz); m_kIzArm = iz; }
   if((ff != m_kFFArm)) { m_armPIDController.setFF(ff); m_kFFArm = ff; }
   if((max != m_kMaxOutputArm) || (min != m_kMinOutputArm)) { 
     m_armPIDController.setOutputRange(min, max); 
     m_kMinOutputArm = min; m_kMaxOutputArm = max; 
   }
   if((maxV != m_armMaxVel)) { m_armPIDController.setSmartMotionMaxVelocity(maxV,0); m_armMaxVel = maxV; }
   if((minV != m_armMinVel)) { m_armPIDController.setSmartMotionMinOutputVelocity(minV,0); m_armMinVel = minV; }
   if((maxA != m_armMaxAcc)) { m_armPIDController.setSmartMotionMaxAccel(maxA,0); m_armMaxAcc = maxA; }
   if((allE != m_armAllowedErr)) { m_armPIDController.setSmartMotionAllowedClosedLoopError(allE,0); m_armAllowedErr = allE; }

   double setPoint, processVariable;
   boolean mode = SmartDashboard.getBoolean("Mode", false);
   if(mode) {
     setPoint = SmartDashboard.getNumber("Set Velocity", 0);
     m_armPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
     processVariable = m_armEncoder.getVelocity();
   } else {
     setPoint = SmartDashboard.getNumber("Set Position", 0);
     // Make sure we don't over-rotate the arm
     setPoint = Math.max(Math.min(setPoint,Constants.ARM_MAX_ANGLE),Constants.ARM_MIN_ANGLE);
     /**
      * As with other PID modes, Smart Motion is set by calling the
      * setReference method on an existing pid object and setting
      * the control type to kSmartMotion
      */
    //TODO: this setReference stuff should be put into the setArmAngle method
    if(m_armTooFar)
      m_armMotorLeader.stopMotor();// double check if the arm goes too far
    else 
      m_armPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    processVariable = m_armEncoder.getPosition();
   }
   
   SmartDashboard.putNumber("SetPoint", setPoint);
   SmartDashboard.putNumber("Process Variable", processVariable);
   SmartDashboard.putNumber("Output", m_armMotorLeader.getAppliedOutput());
 }
}