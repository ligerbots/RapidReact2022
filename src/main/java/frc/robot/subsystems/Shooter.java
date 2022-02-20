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
    CANSparkMax m_chuteMotor;
    //WPI_TalonFX for the shooter
    WPI_TalonFX m_topShooterMotor, m_bottomShooterMotor;
    //Limit Switch for Intake
    DigitalInput m_limitSwitch1, m_limitSwitch2;

    
    //Shooter class constructor, initialize arrays for motors controllers, encoders, and SmartDashboard data
    public Shooter(Vision vision) {

        m_chuteMotor = new CANSparkMax(Constants.CHUTE_CAN_ID, MotorType.kBrushless);

        m_topShooterMotor = new WPI_TalonFX(Constants.TOP_SHOOTER_CAN_ID);
        m_bottomShooterMotor = new WPI_TalonFX(Constants.BOTTOM_SHOOTER_CAN_ID);

        m_limitSwitch1 = new DigitalInput(Constants.LIMIT_SWITCH_ONE);
        m_limitSwitch2 = new DigitalInput(Constants.LIMIT_SWITCH_TWO);

    }

    //periodically update the values of motors for shooter to SmartDashboard
    @Override
    public void periodic() {
    }

    //shoot the ball into the high hub for certain distance
    public void shoot(double top, double bottom, double chute) {
        m_topShooterMotor.set(top);
        m_bottomShooterMotor.set(bottom);
        m_chuteMotor.set(chute);
    }

    //dump the balls into the low hub
    public void dump(){

    }


}
