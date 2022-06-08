// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.FourBallLower;
import frc.robot.commands.OneBallAuto;
import frc.robot.commands.ThreeBallMiddle;
import frc.robot.commands.TrajectoryPlotter;
import frc.robot.commands.TwoBallAutoCurved;
import frc.robot.commands.TwoBallAutoStraight;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private TrajectoryPlotter m_plotter;
  private SendableChooser<AutoCommandInterface> m_chosenAuto = new SendableChooser<>();
  private AutoCommandInterface m_prevAutoCommand = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // start logging data
    // TODO: figure out how this works
    DataLogManager.start("RapidReact2022/dataLogging");
    System.out.println(DataLogManager.getLogDir());
    DataLog log = DataLogManager.getLog();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    // pass the data log into the robotContainer for subsystems to log in data
    m_robotContainer = new RobotContainer(log);

    m_chosenAuto.setDefaultOption("TwoBallAutoStraight", 
      new TwoBallAutoStraight(m_robotContainer.getShooter(), m_robotContainer.getIntake(), m_robotContainer.getDriveTrain(), m_robotContainer.getVision())
    );
    m_chosenAuto.addOption("TwoBallAutoCurved", 
      new TwoBallAutoCurved(m_robotContainer.getShooter(), m_robotContainer.getIntake(), m_robotContainer.getDriveTrain(), m_robotContainer.getVision())
    );
    m_chosenAuto.addOption("OneBallAuto", 
      new OneBallAuto(m_robotContainer.getShooter(), m_robotContainer.getIntake(), m_robotContainer.getDriveTrain(), m_robotContainer.getVision())
    );
    m_chosenAuto.addOption("ThreeBallMiddle", 
      new ThreeBallMiddle(m_robotContainer.getShooter(), m_robotContainer.getIntake(), m_robotContainer.getDriveTrain(), m_robotContainer.getVision())
    );
    m_chosenAuto.addOption("FourBallLower", 
      new FourBallLower(m_robotContainer.getShooter(), m_robotContainer.getIntake(), m_robotContainer.getDriveTrain(), m_robotContainer.getVision())
    );
    SmartDashboard.putData("Chosen Auto", m_chosenAuto);

    m_plotter = new TrajectoryPlotter(m_robotContainer.getDriveTrain().getField2d());

    // Set climber motors to coast so we can move them if we need to.
    m_robotContainer.getClimber().setBrakeMode(true);

    SmartDashboard.putNumber("Constants/SetElevatorHeightTest", 0.0);
    SmartDashboard.putNumber("Constants/SetArmAngleTest", 80.0);
    
    SmartDashboard.putNumber("Constants/OneElevatorIndex", 0.0);
    SmartDashboard.putNumber("Constants/SetOneElevatorHeightTest", 0.0);

    SmartDashboard.putNumber("Constants/AdjustRobotAngleTest", 0.0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.getDriveTrain().setMotorMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {
    AutoCommandInterface autoCommandInterface = m_chosenAuto.getSelected();
    if (autoCommandInterface != null && autoCommandInterface != m_prevAutoCommand) {
      m_robotContainer.getDriveTrain().setPose(autoCommandInterface.getInitialPose());
      m_prevAutoCommand = autoCommandInterface;

      if (Robot.isSimulation()) {
        m_plotter.clear();
        autoCommandInterface.plotTrajectory(m_plotter);
      }
    }    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Cancel the DriveCommand so that the joystick can't override this command
    // group
    m_robotContainer.getDriveCommand().cancel();

    m_robotContainer.getDriveTrain().setMotorMode(NeutralMode.Brake);

    // schedule the autonomous command
    m_autonomousCommand = m_chosenAuto.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDriveCommand().schedule();
    m_robotContainer.getDriveTrain().setMotorMode(NeutralMode.Brake);

    // Set Climber motors to Brake mode
    m_robotContainer.getClimber().setBrakeMode(true);
    
    m_robotContainer.getClimber().m_elevator[0].resetElevatorPos();
    m_robotContainer.getClimber().m_elevator[1].resetElevatorPos(); 

    m_robotContainer.getClimber().m_arm[0].resetArmPos();
    m_robotContainer.getClimber().m_arm[1].resetArmPos();  
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
