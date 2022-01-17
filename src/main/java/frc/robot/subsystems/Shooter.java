/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;
import java.util.TreeMap;
import java.util.Map.Entry;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// suppress unneeded warnings about serialVersionUID in TreeMap
@SuppressWarnings("serial")
public class Shooter extends SubsystemBase {

    public Shooter() {

    }

    @Override
    public void periodic() {
    }

    public double getVoltage() {
        return 0.0;
    }

    public void setHood(double angle) {

    }

    public double getSpeed() {
        return 0.0;
    }

    public void prepareShooter(double distance) {
        // Set the shooter and hood based on the distance
        setShooterRpm(calculateShooterSpeed(distance));
        setHood(calculateShooterHood(distance));
    }

    // public void setShooterVoltage (double voltage) {
    //     pidController.setReference(voltage, ControlType.kVoltage);
    // }

    public void shoot() {

    }

    public void setShooterRpm(double rpm) {

    }

    public double calculateShooterSpeed(double distance) {
        return 0.0;
    }

    public double calculateShooterHood(double distance) {
        return 0.0;
    }

    public void warmUp() {

    }

    public boolean speedOnTarget(final double targetVelocity, final double percentAllowedError) {
        return true;
    }

    public boolean hoodOnTarget(final double targetAngle) {
        return true;
    }

    public void calibratePID(final double p, final double i, final double d, final double f) {
 
    }

    public void stopAll() {
        
    }

    public double getTurretAngle() {
        return 0.0;
    }

    private void setTurret(double angle) {
       
    }

    public void setTurretAdjusted(double adjustedAngle) {
    }
}
