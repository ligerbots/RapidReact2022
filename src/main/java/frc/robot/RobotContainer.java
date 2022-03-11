// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbToNextBar;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RaiseToBar;
import frc.robot.commands.ResetElevatorEncoder;
import frc.robot.commands.SetArmAngleTest;
import frc.robot.commands.SetArmBrake;
import frc.robot.commands.SetArmCoast;
import frc.robot.commands.SetClimber;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetElevatorHeightTest;
import frc.robot.commands.TuneShooterCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    XboxController m_xbox = new XboxController(0);
    Joystick m_farm = new Joystick(1);

    // The robot's subsystems and commands are defined here...
    private final DriveTrain m_driveTrain = new DriveTrain();
    private final Vision m_vision = new Vision(m_driveTrain);
    private final Climber m_climber = new Climber();
    private final Shooter m_shooter = new Shooter();
    private final Intake m_intake = new Intake();
    
    private final DriveCommand m_driveCommand = new DriveCommand(m_driveTrain, new Throttle(), new Turn());

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // only used for tuning
        JoystickButton xboxAButton = new JoystickButton(m_xbox, Constants.XBOX_A);
        xboxAButton.whileHeld(new TuneShooterCommand(m_shooter, m_intake));

        // actual shooter command
        JoystickButton xboxXButton = new JoystickButton(m_xbox, Constants.XBOX_X);
        xboxXButton.whenPressed(new ShooterCommand(m_shooter, m_intake, m_vision, true));//shooting for upperHub

        JoystickButton xboxYButton = new JoystickButton(m_xbox, Constants.XBOX_Y);
        xboxYButton.whenPressed(new ShooterCommand(m_shooter, m_intake, m_vision, false));//shooting for lowerHub

        JoystickButton bumperRight = new JoystickButton(m_xbox, Constants.XBOX_RB);
        bumperRight.whileHeld(new IntakeCommand(m_intake, Constants.INTAKE_SPEED));
    
        JoystickButton bumperLeft = new JoystickButton(m_xbox, Constants.XBOX_LB);
        bumperLeft.whileHeld(new IntakeCommand(m_intake, -Constants.INTAKE_SPEED));

        // farm controller
        JoystickButton farm1 = new JoystickButton(m_farm, 1);
        farm1.whenPressed(new SetElevatorHeightTest(m_climber, "Constants/SetElevatorHeightTest"));

        JoystickButton farm2 = new JoystickButton(m_farm, 2);
        farm2.whenPressed(new SetArmAngleTest(m_climber, "Constants/SetArmAngleTest"));

        JoystickButton farm6 = new JoystickButton(m_farm, 6);
        farm6.whenPressed(new SetClimber(m_climber));

        JoystickButton farm7 = new JoystickButton(m_farm, 7);
        farm7.whenPressed(new RaiseToBar(m_climber));

        JoystickButton farm8 = new JoystickButton(m_farm, 8);
        farm8.whenPressed(new ClimbToNextBar(m_climber));

        JoystickButton farm11 = new JoystickButton(m_farm, 11);
        farm11.whenPressed(new ResetElevatorEncoder(m_climber));

        JoystickButton farm13 = new JoystickButton(m_farm, 13);
        farm13.whenPressed(new SetArmCoast(m_climber));

        JoystickButton farm15 = new JoystickButton(m_farm, 15);
        farm15.whenPressed(new SetArmBrake(m_climber));
    }

    private class Throttle implements DoubleSupplier {
        @Override
        public double getAsDouble() {
            // the controller does <0 is forward
            return -m_xbox.getLeftY();
        }
    }

    private class Turn implements DoubleSupplier {
        @Override
        public double getAsDouble() {
            return -m_xbox.getRightX();
        }
    }

    /*
     * Getters for Commands and Subsystems. Notice that it's public, meaning that
     * outsiders can access it.
     */
    
    public DriveCommand getDriveCommand() {
        return m_driveCommand;
    }

    public DriveTrain getDriveTrain(){
        return m_driveTrain;
    }
    
    public Vision getVision() {
        return m_vision;
    }
    
    public Climber getClimber(){
        return m_climber;
    }

    public Shooter getShooter(){
        return m_shooter;
    }

    public Intake getIntake(){
        return m_intake;
    }

    
    // LigerBots: we don't use this function.
    // Autonomous is controlled by a Chooser defined in Robot.
    //
    // public Command getAutonomousCommand() {
    // return null;
    // }
}