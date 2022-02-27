/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Map;
import java.util.TreeMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    // CANSparkMax for the hopper
    CANSparkMax m_chuteMotor;
    // WPI_TalonFX for the shooter
    WPI_TalonFX m_topShooterMotor, m_bottomShooterMotor;
    // Limit Switch for Intake
    // TODO: change to color sensor
    DigitalInput m_limitSwitch1, m_limitSwitch2;

    static TreeMap<Double, ShooterSpeeds> shooterSpeeds = new TreeMap<>(Map.ofEntries(
            Map.entry(1., new ShooterSpeeds(0.5, 0.5, 0.5)),
            Map.entry(2., new ShooterSpeeds(0.5, 0.5, 0.5)),
            Map.entry(3., new ShooterSpeeds(0.5, 0.5, 0.5))
            ));

    // Shooter class constructor, initialize arrays for motors controllers,
    // encoders, and SmartDashboard data
    public Shooter() {
        m_chuteMotor = new CANSparkMax(Constants.CHUTE_CAN_ID, MotorType.kBrushless);

        m_topShooterMotor = new WPI_TalonFX(Constants.TOP_SHOOTER_CAN_ID);
        m_bottomShooterMotor = new WPI_TalonFX(Constants.BOTTOM_SHOOTER_CAN_ID);

        m_limitSwitch1 = new DigitalInput(Constants.LIMIT_SWITCH_ONE);
        m_limitSwitch2 = new DigitalInput(Constants.LIMIT_SWITCH_TWO);

        // Config the Velocity closed loop gains in slot0
        m_topShooterMotor.config_kP(0, Constants.SHOOTER_KP);
        m_topShooterMotor.config_kI(0, Constants.SHOOTER_KI);
        m_topShooterMotor.config_kD(0, Constants.SHOOTER_KD);
        m_topShooterMotor.config_kF(0, Constants.SHOOTER_KF);

        m_bottomShooterMotor.config_kP(0, Constants.SHOOTER_KP);
        m_bottomShooterMotor.config_kI(0, Constants.SHOOTER_KI);
        m_bottomShooterMotor.config_kD(0, Constants.SHOOTER_KD);
        m_bottomShooterMotor.config_kF(0, Constants.SHOOTER_KF);
    }

    public static class ShooterSpeeds {
        public double top, bottom, chute;
        
        public ShooterSpeeds(double top, double bottom, double chute) {
            this.top = top;
            this.bottom = bottom;
            this.chute = chute;
        }

        public ShooterSpeeds interpolate(ShooterSpeeds other, double ratio) {
            return new ShooterSpeeds(
                    top + (other.top - top) * ratio,
                    bottom + (other.bottom - bottom) * ratio,
                    chute + (other.chute - chute) * ratio);
        }
    }

    public static ShooterSpeeds calculateShooterSpeeds(double distance) {
        Map.Entry<Double, ShooterSpeeds> before = shooterSpeeds.floorEntry(distance);
        Map.Entry<Double, ShooterSpeeds> after = shooterSpeeds.ceilingEntry(distance);
        if (before == null && after == null)
            return new ShooterSpeeds(0, 0, 0); // this should never happen b/c shooterSpeeds should have at least 1 element
        if (before == null)
            return after.getValue();
        if (after == null)
            return before.getValue();
        double ratio = (distance - before.getKey()) / (after.getKey() - before.getKey());
        return before.getValue().interpolate(after.getValue(), ratio);
    }

    // periodically update the values of motors for shooter to SmartDashboard
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/bottom_rpm", getBottomRpm());
        SmartDashboard.putNumber("shooter/top_rpm", getTopRpm());
    }

    public double getTopRpm() {
        return m_topShooterMotor.getSelectedSensorVelocity() / Constants.FALCON_UNITS_PER_RPM;
    }

    public double getBottomRpm() {
        return m_bottomShooterMotor.getSelectedSensorVelocity() / Constants.FALCON_UNITS_PER_RPM;
    }

    public void setShooterRpms(double topRpm, double bottomRpm) {
        // double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0; //RPM -> Native units
        // double targetVelocity_UnitsPer100ms = leftYstick * 2000.0 * 2048.0 / 600.0;
        double falconTop = topRpm * Constants.FALCON_UNITS_PER_RPM;
        double falconBottom = bottomRpm * Constants.FALCON_UNITS_PER_RPM;
        System.out.println("setting shooter " + falconTop + " " + falconBottom);
        m_topShooterMotor.set(ControlMode.Velocity, falconTop);
        m_bottomShooterMotor.set(TalonFXControlMode.Velocity, falconBottom);
    }

    public void setChuteSpeed(double chute) {
        m_chuteMotor.set(chute);
    }
}
