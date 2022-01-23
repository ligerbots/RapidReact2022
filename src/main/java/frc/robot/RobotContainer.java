// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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

  public final Vision vision = new Vision(m_driveTrain);
  public final Intake intake = new Intake();
  public final Shooter shooter = new Shooter(vision);
  public final Climber climber = new Climber();
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
  private void configureButtonBindings() {}
  private class Throttle implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      return m_xbox.getLeftY();
    }
  }

  private class Turn implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      return m_xbox.getRightX();
    }
  }

  /* The getter for m_driveCommand. Notice that it's public, meaning that outsiders can access it. */
  public DriveCommand getDriveCommand(){
    return m_driveCommand;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  //}
}
