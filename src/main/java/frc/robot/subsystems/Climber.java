// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

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

  public CANSparkMax[] m_elevatorMotor;
  
  public SparkMaxLimitSwitch[] m_limitSwitch;


  public Climber() {
    // Construct the arm trapezoid subsystems
    m_arm[0] = new ClimberArm(0, false);
    m_arm[1] = new ClimberArm(1, true);
    m_elevatorMotor = new CANSparkMax[] {new CANSparkMax(Constants.ELEVATOR_CAN_IDS[0], MotorType.kBrushless), new CANSparkMax(Constants.ELEVATOR_CAN_IDS[1], MotorType.kBrushless)};
    
    m_elevatorAscend[0] = new ElevatorAscend(0, false, this, m_elevatorMotor[0]);
    m_elevatorAscend[1] = new ElevatorAscend(1, true, this, m_elevatorMotor[1]);
    m_elevatorDescend[0] = new ElevatorDescend(0, false, this, m_elevatorMotor[0]);
    m_elevatorDescend[1] = new ElevatorDescend(1, true, this, m_elevatorMotor[1]);

    SmartDashboard.putNumber("arm/goal", m_armGoal);
    SmartDashboard.putNumber("elevator/goal", m_elevatorGoal);

    SmartDashboard.putNumber("ElevatorIndex", 0);
    SmartDashboard.putNumber("SetOneElevatorHeightTest", 0.0);

    m_limitSwitch = new SparkMaxLimitSwitch[2];

    m_limitSwitch[0] = m_elevatorMotor[0].getReverseLimitSwitch(Type.kNormallyOpen);
    m_limitSwitch[0].enableLimitSwitch(true);
    m_limitSwitch[1] = m_elevatorMotor[1].getReverseLimitSwitch(Type.kNormallyOpen);
    m_limitSwitch[1].enableLimitSwitch(true);
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

    SmartDashboard.putBoolean("elevatorAscending" + 0, m_elevatorAscend[0].m_elevatorAscending);
    SmartDashboard.putBoolean("elevatorAscending" + 1, m_elevatorAscend[1].m_elevatorAscending);

    SmartDashboard.putBoolean("elevatorDescending" + 0, m_elevatorDescend[0].m_elevatorDescending);
    SmartDashboard.putBoolean("elevatorDescending" + 1, m_elevatorDescend[1].m_elevatorDescending);    

    SmartDashboard.putBoolean("if0", m_limitSwitch[0].isPressed() && m_elevatorDescend[0].m_elevatorDescending);
    SmartDashboard.putBoolean("if1", m_limitSwitch[1].isPressed() && m_elevatorDescend[1].m_elevatorDescending);

    SmartDashboard.putBoolean("elevator0/limitSwitchPressed", m_limitSwitch[0].isPressed());
    SmartDashboard.putBoolean("elevator0/limitActive", m_limitSwitch[0].isPressed() && getElevatorHeight()[0] < Constants.ELEVATOR_CHECKING_LIMIT_SWITCH_HEIGHT);
    // Should be OK because the limit switch is being checked in SetElevatorHeight() and SetElevatorHeightTest()
    // if (m_limitSwitch[0].isPressed() && m_elevatorDescend[0].m_elevatorDescending) {
    //   m_elevatorMotor[0].getEncoder().setPosition(Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT);
    //   setElevatorHeight(0, 0.0);
    // }

    SmartDashboard.putBoolean("elevator1/limitSwitchPressed", m_limitSwitch[1].isPressed());
    SmartDashboard.putBoolean("elevator1/limitActive", m_limitSwitch[1].isPressed() && getElevatorHeight()[1] < Constants.ELEVATOR_CHECKING_LIMIT_SWITCH_HEIGHT);
    // if (m_limitSwitch[1].isPressed() && m_elevatorDescend[1].m_elevatorDescending) {
    //   m_elevatorMotor[1].getEncoder().setPosition(Constants.ELEVATOR_LIMIT_SWITCH_HEIGHT);
    //   setElevatorHeight(1, 0.0);
    // }
  }

  // sets the elevator to a certain height
  public void setElevatorHeight(double height) {
    // TODO: This can just call setElevatorHeight(int index, double height) for each index.
    double curHeight = getElevatorHeight()[0];//both motors are similar, getting current height  
    if(height > curHeight){
        m_elevatorAscend[0].resetElevatorPos();
        m_elevatorAscend[1].resetElevatorPos();
        m_elevatorAscend[0].elevatorAscending();
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

  // sets one of the elevator to a certain height
  public void setElevatorHeight(int index, double height) {
    double curHeight = getElevatorHeight()[index];  
    if(height > curHeight){
        m_elevatorAscend[index].resetElevatorPos();
        m_elevatorAscend[index].elevatorAscending();
        m_elevatorAscend[index].setGoal(height);
      }else{
        m_elevatorDescend[index].resetElevatorPos();
        m_elevatorDescend[index].elevatorDescending();
        m_elevatorDescend[index].setGoal(height);
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