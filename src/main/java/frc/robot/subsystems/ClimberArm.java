// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class ClimberArm extends TrapezoidProfileSubsystem {

  // Define the motor and encoders
  private final CANSparkMax m_motor;
  private final RelativeEncoder m_encoder;
  private final SparkMaxPIDController m_PIDController;

  private final ArmFeedforward m_Feedforward = 
    new ArmFeedforward(Constants.ARM_KS, Constants.ARM_KG, Constants.ARM_KV, Constants.ARM_KA);

  private double m_kPArm = Constants.ARM_K_P;
  private int m_index;
  
  /** Creates a new ClimberArm. */
  public ClimberArm(int index, boolean inverted) {

    super(
      // The constraints for the generated profiles
      new TrapezoidProfile.Constraints(Constants.ARM_MAX_VEL_RAD_PER_SEC, Constants.ARM_MAX_ACC_RAD_PER_SEC_SQ),
      // The initial position of the mechanism
      Constants.ARM_OFFSET_RAD);

    m_index = index;

    // Create the motor, PID Controller and encoder.
    m_motor = new CANSparkMax(Constants.ARM_CAN_IDS[m_index], MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_PIDController = m_motor.getPIDController();
    m_PIDController.setP(m_kPArm);
    m_PIDController.setI(Constants.ARM_K_I);
    m_PIDController.setD(Constants.ARM_K_D);
    m_PIDController.setFF(Constants.ARM_K_FF);
    m_encoder = m_motor.getEncoder();
    // Set the position conversion factor. Note that the Trapezoidal control
    // expects angles in radians.
    m_encoder.setPositionConversionFactor((1.0 / (25.0 * 60.0 / 16.0)) * 2.0 * Math.PI);
    

    SmartDashboard.putNumber("arm/P Gain", Constants.ARM_K_P);
  }

  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    // Calculate the feedforward fromteh setPoint
    double feedforward = m_Feedforward.calculate(setPoint.position, setPoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    // The ArmFeedForward computes in radians. We need to convert back to degrees.
    // Remember that the encoder was already set to account for the gear ratios.
    m_PIDController.setReference(setPoint.position, ControlType.kPosition, 0, feedforward / 12.0);
  }

   @Override
  public void periodic() {
    // Execute the super class periodic method
    super.periodic();

    // Here we can check the SmartDashboard for any updates to the PIC constants.
    // Note that since this is Trapezoidal control, we only need to set P.
    // Each increment will only change the set point position a little bit.

    checkArmPIDVal();

    // Display current values on the Mart Dashboard
    SmartDashboard.putNumber("arm/Output" + m_index, m_motor.getAppliedOutput());
    SmartDashboard.putNumber("arm/Encoder", m_encoder.getPosition());
  }

  private void checkArmPIDVal() {
    double p = SmartDashboard.getNumber("arm/P Gain", 0);
    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != m_kPArm)) {
      m_PIDController.setP(p);
      m_kPArm = p;
    }
  }

}
