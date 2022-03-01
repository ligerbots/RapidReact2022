// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Elevator extends TrapezoidProfileSubsystem {

  // Define the motor and encoders
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_PIDController;

  private final ElevatorFeedforward m_Feedforward = 
    new ElevatorFeedforward(Constants.ELEVATOR_KS, Constants.ELEVATOR_KG, Constants.ELEVATOR_KV, Constants.ELEVATOR_KA);

  private double m_kPElevator = Constants.ELEVATOR_K_P;
  private int m_index;

  private boolean m_tooHigh = false;
  private boolean m_tooLow = false;
  
  /** Creates a new Elevator. */
  public Elevator(int index, boolean inverted) {

    super(
      // The constraints for the generated profiles
      new TrapezoidProfile.Constraints(Constants.ELEVATOR_MAX_VEL_METER_PER_SEC, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ),
      // The initial position of the mechanism
      Constants.ELEVATOR_OFFSET_METER);

    m_index = index;

    // Create the motor, PID Controller and encoder.
    m_motor = new CANSparkMax(Constants.ELEVATOR_CAN_IDS[m_index], MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(inverted);

    m_PIDController = m_motor.getPIDController();
    m_PIDController.setP(m_kPElevator);
    m_PIDController.setI(Constants.ELEVATOR_K_I);
    m_PIDController.setD(Constants.ELEVATOR_K_D);
    m_PIDController.setFF(Constants.ELEVATOR_K_FF);
    m_encoder = m_motor.getEncoder();
    // Set the position conversion factor.
    m_encoder.setPositionConversionFactor(72.0 / 12.0);

    SmartDashboard.putNumber("elevator" + m_index + "/P Gain", m_kPElevator);
  }

  @Override
  public void periodic() {
    // Display current values on the SmartDashboard
    SmartDashboard.putNumber("elevator" + m_index + "/Output" + m_index, m_motor.getAppliedOutput());
    SmartDashboard.putNumber("elevator" + m_index + "/Encoder" + m_index, m_encoder.getPosition());

    // First check if we've gone too far. If we have, reset the setPoint to the limit.
    m_tooHigh = m_encoder.getPosition() > Constants.ELEVATOR_MAX_HEIGHT;
    SmartDashboard.putBoolean("elevator" + m_index + "/too High", m_tooHigh);
    if (m_tooHigh) {
      m_PIDController.setReference(Constants.ELEVATOR_MAX_HEIGHT, ControlType.kPosition, 0, 0.0);
      return;
    }

    m_tooLow = m_encoder.getPosition() < Constants.ELEVATOR_MIN_HEIGHT;
    SmartDashboard.putBoolean("elevator" + m_index + "/too Low", m_tooLow);
    if (m_tooLow) {
      m_PIDController.setReference(Constants.ELEVATOR_MIN_HEIGHT, ControlType.kPosition, 0, 0.0);
      return;
    }
    
    // Execute the super class periodic method
    super.periodic();

    // Here we can check the SmartDashboard for any updates to the PIC constants.
    // Note that since this is Trapezoidal control, we only need to set P.
    // Each increment will only change the set point position a little bit.

    checkPIDVal();
  }

  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    // Calculate the feedforward from the setPoint
    double feedforward = m_Feedforward.calculate(setPoint.position, setPoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    // Remember that the encoder was already set to account for the gear ratios.
    m_PIDController.setReference(setPoint.position, ControlType.kPosition, 0, feedforward / 12.0);
  }

  private void checkPIDVal() {
    double p = SmartDashboard.getNumber("elevator" + m_index + "/P Gain", 0);
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != m_kPElevator)) {
      m_PIDController.setP(p);
      m_kPElevator = p;
    }
  }

  public CANSparkMax getMotor() {
    return m_motor;
  }
}
