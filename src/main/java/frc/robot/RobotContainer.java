// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AdjustRobotAngle;
import frc.robot.commands.AdjustRobotAngleTest;
import frc.robot.commands.ClimbToNextBar;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurnAndShoot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RaiseToBar;
import frc.robot.commands.ResetClimber;
import frc.robot.commands.SetArmAngleTest;
import frc.robot.commands.SetArmBrake;
import frc.robot.commands.SetArmCoast;
import frc.robot.commands.SetClimber;
import frc.robot.commands.SetElevatorHeightTest;
import frc.robot.commands.SetOneElevatorHeightTest;
import frc.robot.commands.SetVisionMode;
import frc.robot.commands.VacuumMode;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.VisionMode;

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
        // JoystickButton xboxYButton = new JoystickButton(m_xbox, Constants.XBOX_Y);
        // xboxYButton.whileHeld(new TuneShooterCommand(m_shooter, m_intake));

        // FOR TESTING!!
        // DriverStation.silenceJoystickConnectionWarning(true);
        
        // vacuum mode
        JoystickButton xboxYButton = new JoystickButton(m_xbox, Constants.XBOX_Y);
        xboxYButton.whileHeld(new VacuumMode(m_shooter, m_intake));

        // actual shooter command

        // shooting for upperHub
        JoystickButton xboxXButton = new JoystickButton(m_xbox, Constants.XBOX_X);
        xboxXButton.whenPressed(new TurnAndShoot(m_shooter, m_intake, m_driveTrain, m_vision, m_driveCommand));

        // shooting for upperHub from tarmac
        JoystickButton xboxAButton = new JoystickButton(m_xbox, Constants.XBOX_A);
        xboxAButton.whenPressed(new ShooterCommand(m_shooter, m_intake, Constants.TARMAC_DEFAULT_DISTANCE, true));

        // shooting for lowerHub
        JoystickButton xboxBButton = new JoystickButton(m_xbox, Constants.XBOX_B);
        xboxBButton.whenPressed(new ShooterCommand(m_shooter, m_intake, m_vision, false));

        // Intake commands
        
        JoystickButton bumperRight = new JoystickButton(m_xbox, Constants.XBOX_RB);
        bumperRight.whileHeld(new IntakeCommand(m_intake, Constants.INTAKE_SPEED));
    
        JoystickButton bumperLeft = new JoystickButton(m_xbox, Constants.XBOX_LB);
        bumperLeft.whileHeld(new IntakeCommand(m_intake, -Constants.INTAKE_SPEED));

        // farm controller
        JoystickButton farm1 = new JoystickButton(m_farm, 1);
        farm1.whenPressed(new SetElevatorHeightTest(m_climber));

        JoystickButton farm2 = new JoystickButton(m_farm, 2);
        farm2.whenPressed(new SetArmAngleTest(m_climber, "Constants/SetArmAngleTest"));

        JoystickButton farm3 = new JoystickButton(m_farm, 3);
        farm3.whenPressed(new SetOneElevatorHeightTest(m_climber));

        JoystickButton farm4 = new JoystickButton(m_farm, 4);
        farm3.whenPressed(new AdjustRobotAngleTest(m_driveTrain));

        JoystickButton farm6 = new JoystickButton(m_farm, 6);
        farm6.whenPressed(new SetClimber(m_climber));

        JoystickButton farm7 = new JoystickButton(m_farm, 7);
        farm7.whenPressed(new RaiseToBar(m_climber).withTimeout(Constants.RAISE_TO_BAR_TIMEOUT));

        JoystickButton farm8 = new JoystickButton(m_farm, 8);
        farm8.whenPressed(new ClimbToNextBar(m_climber).withTimeout(Constants.CLIMB_TO_NEXT_BAR_TIMEOUT));

        JoystickButton farm11 = new JoystickButton(m_farm, 11);
        farm11.whenPressed(new ResetClimber(m_climber));

        JoystickButton farm13 = new JoystickButton(m_farm, 13);
        farm13.whenPressed(new SetArmCoast(m_climber));

        JoystickButton farm15 = new JoystickButton(m_farm, 15);
        farm15.whenPressed(new SetArmBrake(m_climber));        
        
        //Bind buttons for vision modes.
        JoystickButton farm12 = new JoystickButton(m_farm, 12);
        farm12.whenPressed(new SetVisionMode(m_vision, VisionMode.INTAKE)); 

        JoystickButton farm14 = new JoystickButton(m_farm, 14);
        farm14.whenPressed(new SetVisionMode(m_vision, VisionMode.SHOOTER)); 

        JoystickButton farm16 = new JoystickButton(m_farm, 16);
        farm16.whenPressed(new SetVisionMode(m_vision, VisionMode.HUBFINDER)); 

        // For Testing
        // JoystickButton farm10 = new JoystickButton(m_farm, 10);
        // xboxYButton.whenPressed(new FaceShootingTarget(m_driveTrain, m_vision, Constants.TURN_TOLERANCE_DEG, m_driveCommand)); 
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
            return -0.75 * m_xbox.getRightX();
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