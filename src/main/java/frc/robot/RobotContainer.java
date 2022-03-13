// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TuneShooterCommand;
import frc.robot.commands.VacuumMode;
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

        //vacuum mode
        JoystickButton xboxBButton = new JoystickButton(m_xbox, Constants.XBOX_B);
        xboxBButton.whileHeld(new VacuumMode(m_shooter, m_intake));

        // actual shooter command
        JoystickButton xboxXButton = new JoystickButton(m_xbox, Constants.XBOX_X);
        xboxXButton.whenPressed(new ShooterCommand(m_shooter, m_intake, m_vision, true));//shooting for upperHub

        JoystickButton xboxYButton = new JoystickButton(m_xbox, Constants.XBOX_Y);
        xboxYButton.whenPressed(new ShooterCommand(m_shooter, m_intake, m_vision, false));//shooting for lowerHub

        JoystickButton bumperRight = new JoystickButton(m_xbox, Constants.XBOX_RB);
        bumperRight.whileHeld(new IntakeCommand(m_intake, Constants.INTAKE_SPEED));
    
        JoystickButton bumperLeft = new JoystickButton(m_xbox, Constants.XBOX_LB);
        bumperLeft.whileHeld(new IntakeCommand(m_intake, -Constants.INTAKE_SPEED));
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