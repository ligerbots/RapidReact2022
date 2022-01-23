// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  XboxController m_xbox = new XboxController(0);

  // The robot's subsystems and commands are defined here...
  private final Throttle m_throttle = new Throttle(); // create an instance of the throttle class. See explaination below
  private final Turn m_turn = new Turn();
  private final DriveTrain m_driveTrain = new DriveTrain(); 
  private final DriveCommand m_driveCommand = new DriveCommand(m_driveTrain, m_throttle, m_turn); 

  public final Vision m_vision = new Vision(m_driveTrain);
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter(m_vision);
  public final Climber m_climber = new Climber();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public class Throttle implements DoubleSupplier{
    @Override 
    public double getAsDouble() {
      return m_xbox.getLeftY(); // use left joystick for throttle
    }
  }

  public class Turn implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      return m_xbox.getRightX(); // use right joystick for turn
    }
  }

  public DriveCommand getDriveCommand(){
    return m_driveCommand;
  }
  private class DriveSwitch implements BooleanSupplier{
    @Override
    public boolean getAsBoolean() {
      return m_xbox.getBButton();
    }
  }
  
  // public class Shoulder implements DoubleSupplier{
  //   @Override
  //   public double getAsDouble() {
  //     //return xbox.getTriggerAxis(Hand.kRight) - xbox.getTriggerAxis(Hand.kLeft);// set shoulder speed 
  //     return 0.0;
  //   }
  // }

  private void configureButtonBindings() {
    if (Robot.isSimulation()) {
      // for the simulation, silence warnings about missing joysticks
      DriverStation.getInstance().silenceJoystickConnectionWarning(true);
    }
  }
  /**
   * LigerBots: we don't use this function. 
   * Autonomous is controlled by a Chooser defined in Robot.
   */
  // public Command getAutonomousCommand() {
  //   return null;
  // }
}