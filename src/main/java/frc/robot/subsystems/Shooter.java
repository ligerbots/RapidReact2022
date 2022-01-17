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

    //Shooter class constructor, initialize arrays for motors controllers, encoders, and SmartDashboard data
    public Shooter() {

    }

    //periodically undate the values of motors for shooter to SmartDashboard
    @Override
    public void periodic() {
    }

    // Set the shooter and hood based on the distance
    public void prepareShooter(double distance) {

    }

    //shoot the ball for high hub
    public void shootHigh() {

    }
    //shoot the ball for low hub
    public void shootLow() {

    }

    //dump the balls for low hub
    public void dump(){

    }
}
