// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // sets the elevator to a certain height
  public void setElevatorHeight(double height) {

  }
  
  // rotates the arms a certain degree
  public void rotateArm(double degree){

  }

  // returns the currrent height of the elevator
  public void getElevatorHeight(){

  }

  //returns the current angle of the arm
  public void getArmAngle(){

  }
}
