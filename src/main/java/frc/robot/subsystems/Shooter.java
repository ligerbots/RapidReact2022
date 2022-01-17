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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Shooter extends SubsystemBase {

    //CANSparkMax for the hopper
    CANSparkMax motor1, motor2;
    //WPI_TalonSRX for the shooter
    WPI_TalonSRX motor3, motor4;
    //Limit Switch for Intake
    DigitalInput limitSwitch1, limitSwitch2;


    //Shooter class constructor, initialize arrays for motors controllers, encoders, and SmartDashboard data
    public Shooter() {

        motor1 = new CANSparkMax(Constants.HOPPER_ONE_CAN_ID, MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.HOPPER_TWO_CAN_ID, MotorType.kBrushless);

        motor3 = new WPI_TalonSRX(Constants.SHOOTER_ONE_CAN_ID);
        motor4 = new WPI_TalonSRX(Constants.SHOOTER_TWO_CAN_ID);

        limitSwitch1 = new DigitalInput(Constants.LIMIT_SWITCH_ONE);
        limitSwitch2 = new DigitalInput(Constants.LIMIT_SWITCH_TWO);
    }

    //periodically update the values of motors for shooter to SmartDashboard
    @Override
    public void periodic() {
    }

    //shoot the ball into the high hub for certain distance
    public void shoot(double distance) {

    }

    //dump the balls into the low hub
    public void dump(){

    }
}
