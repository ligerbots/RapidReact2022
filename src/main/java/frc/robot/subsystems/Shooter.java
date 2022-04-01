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
    // DigitalInput m_limitSwitch1, m_limitSwitch2;

    // lookup table for upper hub speeds
    static final TreeMap<Double, ShooterSpeeds> shooterSpeeds = new TreeMap<>(Map.ofEntries(
            Map.entry(0.0, new ShooterSpeeds(900.0, 900.0, 0.3)),         // actually lower hub, but safer to include
            Map.entry(54.99, new ShooterSpeeds(900.0, 900.0, 0.3)),       // actually lower hub, but safer to include
            Map.entry(55.0, new ShooterSpeeds(1400.0, 1650.0, 0.3)),      // revere
            Map.entry(71.0, new ShooterSpeeds(1450.0, 1700.0, 0.3)),      // shed
            Map.entry(84.0, new ShooterSpeeds(1550.0, 1550.0, 0.3)),      // revere
            Map.entry(92.0, new ShooterSpeeds(1600.0, 1600.0, 0.3)),      // shed
            // Map.entry(102.0, new ShooterSpeeds(1750.0, 1750.0, 0.3)),  // revere
            Map.entry(123.0, new ShooterSpeeds(1650.0, 1750.0, 0.3)),     // revere
            Map.entry(154.0, new ShooterSpeeds(1750.0, 2000.0, 0.3)),     // revere
            Map.entry(163.0, new ShooterSpeeds(1850.0, 2150.0, 0.3)),     // revere
            // Map.entry(190.0, new ShooterSpeeds(1950.0, 2250.0, 0.3)),
            Map.entry(195.0, new ShooterSpeeds(1900.0, 2100.0, 0.3)),     // revere
            Map.entry(235.0, new ShooterSpeeds(2120.0, 2300.0, 0.3))));   // revere
            
    // values for lowerHub
    static final ShooterSpeeds lowHubSpeeds = new ShooterSpeeds(900.0, 900.0, 0.3);

    // Shooter class constructor, initialize arrays for motors controllers,
    // encoders, and SmartDashboard data
    public Shooter() {
        m_chuteMotor = new CANSparkMax(Constants.CHUTE_CAN_ID, MotorType.kBrushless);

        m_topShooterMotor = new WPI_TalonFX(Constants.TOP_SHOOTER_CAN_ID);
        m_bottomShooterMotor = new WPI_TalonFX(Constants.BOTTOM_SHOOTER_CAN_ID);

        // m_limitSwitch1 = new DigitalInput(Constants.LIMIT_SWITCH_ONE);
        // m_limitSwitch2 = new DigitalInput(Constants.LIMIT_SWITCH_TWO);

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
                    top + (other.top - top) * ratio, //get ratiod
                    bottom + (other.bottom - bottom) * ratio,
                    chute + (other.chute - chute) * ratio);
        }
    }

    public static ShooterSpeeds calculateShooterSpeeds(double distance, boolean upperHub) {
        if (upperHub == false) {
            // if shooting to lowerHub, then return shooterSpeed with values for lowerHub
            return lowHubSpeeds;
        }

        Map.Entry<Double, ShooterSpeeds> before = shooterSpeeds.floorEntry(distance);
        Map.Entry<Double, ShooterSpeeds> after = shooterSpeeds.ceilingEntry(distance);
        if (before == null) {
            if (after == null) {
                return lowHubSpeeds; // this should never happen b/c shooterSpeeds should have at least 1 element
            }
            return after.getValue();
        }
        if (after == null)
            return before.getValue();
            
        double denom = after.getKey() - before.getKey();
        if (Math.abs(denom) < 0.1) {
            // distance must have exactly matched a key
            return before.getValue();
        }

        double ratio = (distance - before.getKey()) / denom;
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
        System.out.println("setting shooter motor signals " + falconTop + " " + falconBottom);
        m_topShooterMotor.set(ControlMode.Velocity, falconTop);
        m_bottomShooterMotor.set(TalonFXControlMode.Velocity, falconBottom);
    }

    public void setChuteSpeed(double chute) {
        m_chuteMotor.set(-chute);
    }
}
