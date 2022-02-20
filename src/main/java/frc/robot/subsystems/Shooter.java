/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;


public class Shooter extends SubsystemBase {

    //CANSparkMax for the hopper
    CANSparkMax m_hopper1, m_hopper2;
    //WPI_TalonFX for the shooter
    WPI_TalonFX m_shooterTop, m_shooterBottom;
    //Limit Switch for Intake
    DigitalInput m_limitSwitch1, m_limitSwitch2;

    
    //Shooter class constructor, initialize arrays for motors controllers, encoders, and SmartDashboard data
    public Shooter(Vision vision) {

        m_hopper1 = new CANSparkMax(Constants.HOPPER_ONE_CAN_ID, MotorType.kBrushless);
        m_hopper2 = new CANSparkMax(Constants.HOPPER_TWO_CAN_ID, MotorType.kBrushless);

        m_shooterTop = new WPI_TalonFX(Constants.SHOOTER_ONE_CAN_ID);
        m_shooterBottom = new WPI_TalonFX(Constants.SHOOTER_TWO_CAN_ID);

        m_limitSwitch1 = new DigitalInput(Constants.LIMIT_SWITCH_ONE);
        m_limitSwitch2 = new DigitalInput(Constants.LIMIT_SWITCH_TWO);

    }

    //periodically update the values of motors for shooter to SmartDashboard
    @Override
    public void periodic() {
    }

    //shoot the ball into the high hub for certain distance
    public void shoot(double top, double bottom, double chute) {
        m_shooterTop.set(top);
        m_shooterBottom.set(bottom);
        m_hopper1.set(chute);
        m_hopper2.set(chute);
    }

    //dump the balls into the low hub
    public void dump(){

    }


}
