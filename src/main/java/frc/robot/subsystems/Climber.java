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
  // CANSparkMax m_elevatorMotorLeader;
  // CANSparkMax m_elevatorMotorFollower;
  CANSparkMax[] m_elevatorMotors = new CANSparkMax[2];
  // CANSparkMax m_armMotorLeader;
  // CANSparkMax m_armMotorFollower;
  CANSparkMax[] m_armMotors = new CANSparkMax[2];
  DutyCycleEncoder throughBoreEncoder;
  RelativeEncoder[] m_elevatorEncoders = new RelativeEncoder[2]; // it is used on the winch on the actual robot
  RelativeEncoder[] m_armEncoders = new RelativeEncoder[2];
  SmartDashboard smartDashboard;
  // SparkMaxPIDController m_elevatorPIDController, m_armPIDController;
  SparkMaxPIDController[] m_elevatorPIDControllers = new SparkMaxPIDController[2];
  SparkMaxPIDController[] m_armPIDControllers = new SparkMaxPIDController[2];

  // PID coefficients for Arm
  double m_kPArm = 5e-5;
  double m_kIArm = 1e-6;
  double m_kDArm = 0;
  double m_kIzArm = 0;
  double m_kFFArm = 0.000156;
  double m_kMaxOutputArm = 1;
  double m_kMinOutputArm = -1;

  double m_armMaxRPM = 5700;

  // Smart Motion Coefficients for Arm
  double m_armMaxVel = 2000; // rpm
  double m_armMinVel = 0;
  double m_armMaxAcc = 100;

  double m_armAllowedErr = 0;

  boolean[] m_armTooFar = new boolean[]{false, false};

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
  double m_elevatorMaxAcc = 100;

  double m_elevatorAllowedErr = 0;

  boolean[] m_elevatorTooFar = new boolean[] {false, false};

  double[] m_elevatorEncoderValue = new double[2];
  double[] m_armEncoderValue = new double[2];

  public Climber() {
    m_armMotors[0] = new CANSparkMax(Constants.ARM_LEADER_CAN_ID, MotorType.kBrushless);
    m_armMotors[1] = new CANSparkMax(Constants.ARM_FOLLOWER_CAN_ID, MotorType.kBrushless);
    m_elevatorMotors[0] = new CANSparkMax(Constants.ELEVATOR_LEADER_CAN_ID, MotorType.kBrushless);
    m_elevatorMotors[1] = new CANSparkMax(Constants.ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);

    // This will reset the encoder value to 0
    // m_armMotorLeader.restoreFactoryDefaults();
    // m_armMotorFollower.restoreFactoryDefaults();

    for (int i = 0; i < 2; i++) {
      m_armMotors[i].restoreFactoryDefaults();
      m_elevatorMotors[i].restoreFactoryDefaults();
    }

    // Set right arm to follow left arm and invert
    m_armMotors[1].follow(m_armMotors[0], true);
    // Set right elevator to follow left elevator and invert
    m_elevatorMotors[1].follow(m_elevatorMotors[0], true);

    for (int i = 0; i < 2; i++) {
      m_armPIDControllers[i] = m_armMotors[i].getPIDController();
      m_armEncoders[i] = m_armMotors[i].getEncoder();
      // gear Ratio for Arm is 25::1 Max Planetary and 60::16 chain reduction
      m_armEncoders[i].setPositionConversionFactor((1.0 / (25.0 * 60.0 / 16.0)) * 360.0);
      System.out.println("Get Position Conversion Factor For Arm");
      m_armPIDControllers[i].setP(m_kPArm);
      m_armPIDControllers[i].setI(m_kIArm);
      m_armPIDControllers[i].setD(m_kDArm);
      m_armPIDControllers[i].setIZone(m_kIzArm);
      m_armPIDControllers[i].setFF(m_kFFArm);
      m_armPIDControllers[i].setOutputRange(m_kMinOutputArm, m_kMaxOutputArm);

      // set arm PID smartMotion Coefficients
      int smartMotionSlotArm = 0;
      m_armPIDControllers[i].setSmartMotionMaxVelocity(m_armMaxVel, smartMotionSlotArm);
      m_armPIDControllers[i].setSmartMotionMinOutputVelocity(m_armMinVel, smartMotionSlotArm);
      m_armPIDControllers[i].setSmartMotionMaxAccel(m_armMaxAcc, smartMotionSlotArm);
      m_armPIDControllers[i].setSmartMotionAllowedClosedLoopError(m_armAllowedErr, smartMotionSlotArm);

      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("arm/P Gain", m_kPArm);
      SmartDashboard.putNumber("arm/I Gain", m_kIArm);
      SmartDashboard.putNumber("arm/D Gain", m_kDArm);
      SmartDashboard.putNumber("arm/I Zone", m_kIzArm);
      SmartDashboard.putNumber("arm/Feed Forward", m_kFFArm);
      SmartDashboard.putNumber("arm/Max Output", m_kMaxOutputArm);
      SmartDashboard.putNumber("arm/Min Output", m_kMinOutputArm);

      // display Smart Motion coefficients
      SmartDashboard.putNumber("arm/Max Velocity", m_armMaxVel);
      SmartDashboard.putNumber("arm/Min Velocity", m_armMinVel);
      SmartDashboard.putNumber("arm/Max Acceleration", m_armMaxAcc);
      SmartDashboard.putNumber("arm/Allowed Closed Loop Error", m_armAllowedErr);
      SmartDashboard.putNumber("arm/Set Position", 0);
      SmartDashboard.putNumber("arm/Set Velocity", 0);

      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean("arm/Mode", true);

      m_elevatorPIDControllers[i] = m_elevatorMotors[i].getPIDController();
      m_elevatorEncoders[i] = m_elevatorMotors[i].getEncoder();
      m_elevatorEncoders[i].setPosition(m_elevatorEncoderValue[i]);
      SmartDashboard.putNumber("elevator/Encoder Value"+i, m_elevatorEncoderValue[i]);
      // gear Ratio for Elevator is 72::12, assuming only one wrap of the rope
      m_elevatorEncoders[i].setPositionConversionFactor((12.0 / 72.0) * Math.PI * 0.75);

      System.out.println("Get Position Conversion Factor For Elevator");
      m_elevatorPIDControllers[i].setP(m_kPElevator);
      m_elevatorPIDControllers[i].setI(m_kIElevator);
      m_elevatorPIDControllers[i].setD(m_kDElevator);
      m_elevatorPIDControllers[i].setIZone(m_kIzElevator);
      m_elevatorPIDControllers[i].setFF(m_kFFElevator);
      m_elevatorPIDControllers[i].setOutputRange(m_kMinOutputElevator, m_kMaxOutputElevator);

      // set elevator PID smartMotion Coefficients
      int smartMotionSlotElevator = 0;
      m_elevatorPIDControllers[i].setSmartMotionMaxVelocity(m_elevatorMaxVel, smartMotionSlotElevator);
      m_elevatorPIDControllers[i].setSmartMotionMinOutputVelocity(m_elevatorMinVel, smartMotionSlotElevator);
      m_elevatorPIDControllers[i].setSmartMotionMaxAccel(m_elevatorMaxAcc, smartMotionSlotElevator);
      m_elevatorPIDControllers[i].setSmartMotionAllowedClosedLoopError(m_elevatorAllowedErr, smartMotionSlotElevator);

      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("elevator/P Gain", m_kPElevator);
      SmartDashboard.putNumber("elevator/I Gain", m_kIElevator);
      SmartDashboard.putNumber("elevator/D Gain", m_kDElevator);
      SmartDashboard.putNumber("elevator/I Zone", m_kIzElevator);
      SmartDashboard.putNumber("elevator/Feed Forward", m_kFFElevator);
      SmartDashboard.putNumber("elevator/Max Output", m_kMaxOutputElevator);
      SmartDashboard.putNumber("elevator/Min Output", m_kMinOutputElevator);

      // display Smart Motion coefficients
      SmartDashboard.putNumber("elevator/Max Velocity", m_elevatorMaxVel);
      SmartDashboard.putNumber("elevator/Min Velocity", m_elevatorMinVel);
      SmartDashboard.putNumber("elevator/Max Acceleration", m_elevatorMaxAcc);
      SmartDashboard.putNumber("elevator/Allowed Closed Loop Error", m_elevatorAllowedErr);
      SmartDashboard.putNumber("elevator/Set Position", 0);
      SmartDashboard.putNumber("elevator/Set Velocity", 0);

      // button to toggle between velocity and smart motion modes
      SmartDashboard.putBoolean("elevator/Mode", true);
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run

    for (int i=0; i<2; i++){
    checkArmPIDVal(i);

    // If we go too far in either direction, shut it down
    if (m_armEncoders[i].getPosition() > Constants.ARM_MAX_ANGLE ||
        m_armEncoders[i].getPosition() < Constants.ARM_MIN_ANGLE) {
      m_armMotors[i].stopMotor();
      m_armTooFar[i] = true;
    } else {
      m_armTooFar[i] = false;
    }
    SmartDashboard.putBoolean("arm/Too Far"+i, m_armTooFar[i]);

    // This method will be called once per scheduler run
    checkElevatorPIDVal(i);

    // If we go too far in either direction, shut it down
    /*
     * if (m_elevatorEncoder.getPosition() > Constants.ELEVATOR_MAX_HEIGHT ||
     * m_elevatorEncoder.getPosition() < Constants.ELEVATOR_MIN_HEIGHT) {
     * m_elevatorMotorLeader.stopMotor();
     * m_elevatorTooFar = true;
     * } else {
     * m_elevatorTooFar = false;
     * }
     */
    SmartDashboard.putBoolean("elevator/Too Far"+i, m_elevatorTooFar[i]);

    if (SmartDashboard.getNumber("elevator/Encoder Value"+i, 0) != m_elevatorEncoderValue[i]) {
      m_elevatorEncoderValue[i] = SmartDashboard.getNumber("elevator/Encoder Value"+i, 0);
      m_elevatorEncoders[i].setPosition(m_elevatorEncoderValue[i]);

    }
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
    m_armMotors[0].setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_armMotors[1].setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevatorMotors[0].setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    m_elevatorMotors[1].setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  private void checkArmPIDVal(int j) {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("arm/P Gain", 0);
    double i = SmartDashboard.getNumber("arm/I Gain", 0);
    double d = SmartDashboard.getNumber("arm/D Gain", 0);
    double iz = SmartDashboard.getNumber("arm/I Zone", 0);
    double ff = SmartDashboard.getNumber("arm/Feed Forward", 0);
    double max = SmartDashboard.getNumber("arm/Max Output", 0);
    double min = SmartDashboard.getNumber("arm/Min Output", 0);
    double maxV = SmartDashboard.getNumber("arm/Max Velocity", 0);
    double minV = SmartDashboard.getNumber("arm/Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("arm/Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("arm/Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != m_kPArm)) {
      m_armPIDControllers[j].setP(p);
      m_kPArm = p;
    }
    if ((i != m_kIArm)) {
      m_armPIDControllers[j].setI(i);
      m_kIArm = i;
    }
    if ((d != m_kDArm)) {
      m_armPIDControllers[j].setD(d);
      m_kDArm = d;
    }
    if ((iz != m_kIzArm)) {
      m_armPIDControllers[j].setIZone(iz);
      m_kIzArm = iz;
    }
    if ((ff != m_kFFArm)) {
      m_armPIDControllers[j].setFF(ff);
      m_kFFArm = ff;
    }
    if ((max != m_kMaxOutputArm) || (min != m_kMinOutputArm)) {
      m_armPIDControllers[j].setOutputRange(min, max);
      m_kMinOutputArm = min;
      m_kMaxOutputArm = max;
    }
    if ((maxV != m_armMaxVel)) {
      m_armPIDControllers[j].setSmartMotionMaxVelocity(maxV, 0);
      m_armMaxVel = maxV;
    }
    if ((minV != m_armMinVel)) {
      m_armPIDControllers[j].setSmartMotionMinOutputVelocity(minV, 0);
      m_armMinVel = minV;
    }
    if ((maxA != m_armMaxAcc)) {
      m_armPIDControllers[j].setSmartMotionMaxAccel(maxA, 0);
      m_armMaxAcc = maxA;
    }
    if ((allE != m_armAllowedErr)) {
      m_armPIDControllers[j].setSmartMotionAllowedClosedLoopError(allE, 0);
      m_armAllowedErr = allE;
    }

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("arm/Mode", false);
    if (mode) {
      setPoint = SmartDashboard.getNumber("arm/Set Velocity", 0);
      m_armPIDControllers[j].setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = m_armEncoders[j].getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("arm/Set Position", 0);
      // Make sure we don't over-rotate the arm
      setPoint = Math.max(Math.min(setPoint, Constants.ARM_MAX_ANGLE), Constants.ARM_MIN_ANGLE);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      // TODO: this setReference stuff should be put into the setArmAngle method
      if (m_armTooFar[j])
        m_armMotors[j].stopMotor();// double check if the arm goes too far
      else
        m_armPIDControllers[j].setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = m_armEncoders[j].getPosition();
    }

    SmartDashboard.putNumber("arm/SetPoint", setPoint);
    SmartDashboard.putNumber("arm/Process Variable"+j, processVariable);
    SmartDashboard.putNumber("arm/Output"+j, m_armMotors[j].getAppliedOutput());
  }

  private void checkElevatorPIDVal(int j) {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("elevator/P Gain", 0);
    double i = SmartDashboard.getNumber("elevator/I Gain", 0);
    double d = SmartDashboard.getNumber("elevator/D Gain", 0);
    double iz = SmartDashboard.getNumber("elevator/I Zone", 0);
    double ff = SmartDashboard.getNumber("elevator/Feed Forward", 0);
    double max = SmartDashboard.getNumber("elevator/Max Output", 0);
    double min = SmartDashboard.getNumber("elevator/Min Output", 0);
    double maxV = SmartDashboard.getNumber("elevator/Max Velocity", 0);
    double minV = SmartDashboard.getNumber("elevator/Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("elevator/Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("elevator/Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != m_kPElevator)) {
      m_elevatorPIDControllers[j].setP(p);
      m_kPElevator = p;
    }
    if ((i != m_kIElevator)) {
      m_elevatorPIDControllers[j].setI(i);
      m_kIElevator = i;
    }
    if ((d != m_kDElevator)) {
      m_elevatorPIDControllers[j].setD(d);
      m_kDElevator = d;
    }
    if ((iz != m_kIzElevator)) {
      m_elevatorPIDControllers[j].setIZone(iz);
      m_kIzElevator = iz;
    }
    if ((ff != m_kFFElevator)) {
      m_elevatorPIDControllers[j].setFF(ff);
      m_kFFElevator = ff;
    }
    if ((max != m_kMaxOutputElevator) || (min != m_kMinOutputElevator)) {
      m_elevatorPIDControllers[j].setOutputRange(min, max);
      m_kMinOutputElevator = min;
      m_kMaxOutputElevator = max;
    }
    if ((maxV != m_elevatorMaxVel)) {
      m_elevatorPIDControllers[j].setSmartMotionMaxVelocity(maxV, 0);
      m_elevatorMaxVel = maxV;
    }
    if ((minV != m_elevatorMinVel)) {
      m_elevatorPIDControllers[j].setSmartMotionMinOutputVelocity(minV, 0);
      m_elevatorMinVel = minV;
    }
    if ((maxA != m_elevatorMaxAcc)) {
      m_elevatorPIDControllers[j].setSmartMotionMaxAccel(maxA, 0);
      m_elevatorMaxAcc = maxA;
    }
    if ((allE != m_elevatorAllowedErr)) {
      m_elevatorPIDControllers[j].setSmartMotionAllowedClosedLoopError(allE, 0);
      m_elevatorAllowedErr = allE;
    }

    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("elevator/Mode", false);
    if (mode) {
      setPoint = SmartDashboard.getNumber("elevator/Set Velocity", 0);
      m_elevatorPIDControllers[j].setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      processVariable = m_elevatorEncoders[j].getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("elevator/Set Position", 0);
      // Make sure we don't over- extend/retract the elevator
      setPoint = Math.max(Math.min(setPoint, Constants.ELEVATOR_MAX_HEIGHT), Constants.ELEVATOR_MIN_HEIGHT);
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
      // TODO: this setReference stuff should be put into the setElevatorHeight method
      if (m_elevatorTooFar[j])
        m_elevatorMotors[j].stopMotor();// double check if the elevator goes too far
      else
        m_elevatorPIDControllers[j].setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      processVariable = m_elevatorEncoders[j].getPosition();
    }

    SmartDashboard.putNumber("elevator/SetPoint", setPoint);
    SmartDashboard.putNumber("elevator/Process Variable"+j, processVariable);
    SmartDashboard.putNumber("elevator/Output"+j, m_elevatorMotors[j].getAppliedOutput());
  }
}