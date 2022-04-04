// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.FieldInformation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallLower extends SequentialCommandGroup implements AutoCommandInterface {
  /** Creates a new FourBallLower. */
  static final double DISTANCE_BACK = 1.4;  
  Trajectory m_trajectory1;
  Trajectory m_trajectory2;
  Trajectory m_trajectory3;
  Trajectory m_trajectory4;
  public FourBallLower(Shooter shooter, Intake intake, DriveTrain driveTrain, Vision vision, DriveCommand driveCommand) {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

        TrajectoryConfig reverseConfig =
            new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAcceleration)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);
        
        TrajectoryConfig forwardConfig =
        new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAcceleration)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .setReversed(false);

        Pose2d initialPose = getInitialPose();
        Pose2d firstShootingPose = getInitialPose();
        Pose2d finalShootingPose = FieldInformation.middleBlueBall;
        // Pose2d midPose = new Pose2d(
        //     initialPose.getX() - initialPose.getRotation().getCos() * DISTANCE_BACK, 
        //     initialPose.getY() - initialPose.getRotation().getSin() * DISTANCE_BACK, 
        //     initialPose.getRotation());

        m_trajectory1 = TrajectoryGenerator.generateTrajectory(
            initialPose, 
            List.of(),
            FieldInformation.lowerBlueBall,
            reverseConfig
        ); 

        m_trajectory2 = TrajectoryGenerator.generateTrajectory(
            FieldInformation.lowerBlueBall, 
            List.of(),
            firstShootingPose,
            forwardConfig
        ); 

        m_trajectory3 = TrajectoryGenerator.generateTrajectory(
            firstShootingPose, 
            List.of(
                FieldInformation.middleBlueBall.getTranslation()
            ),
            FieldInformation.cornerBlueBall,
            reverseConfig
        ); 

        m_trajectory4 = TrajectoryGenerator.generateTrajectory(
            FieldInformation.cornerBlueBall, 
            List.of(),
            finalShootingPose,
            forwardConfig
        ); 

        RamseteCommand ramsete1 = new RamseteCommand(
            m_trajectory1,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts,
            driveTrain
        );

        RamseteCommand ramsete2 = new RamseteCommand(
            m_trajectory2,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts,
            driveTrain
        );

        RamseteCommand ramsete3 = new RamseteCommand(
            m_trajectory3,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts,
            driveTrain
        );

        RamseteCommand ramsete4 = new RamseteCommand(
            m_trajectory4,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveTrain::tankDriveVolts,
            driveTrain
        );
    
    addCommands(
        new DeployIntake(driveTrain), 
        new ParallelDeadlineGroup(
            ramsete1.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new IntakeCommand(intake, Constants.INTAKE_SPEED)
        ),
        new ParallelDeadlineGroup(
            ramsete2.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new IntakeCommand(intake, Constants.INTAKE_SPEED)
        ),
        // new FaceShootingTarget(driveTrain, vision, Constants.TURN_TOLERANCE_DEG, driveCommand),
        // new ShooterCommand(shooter, intake, vision, true),
        new ParallelDeadlineGroup(
            ramsete3.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new IntakeCommand(intake, Constants.INTAKE_SPEED)
        ),
        new ParallelDeadlineGroup(
            ramsete4.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new IntakeCommand(intake, Constants.INTAKE_SPEED)
        ),
        new FaceShootingTarget(driveTrain, vision, Constants.TURN_TOLERANCE_DEG, driveCommand),
        new ShooterCommand(shooter, intake, vision, true)
    );
  }

  @Override
  public Pose2d getInitialPose() {
      return FieldInformation.lowerBlueStart;
  }
  
  @Override
  public void plotTrajectory(TrajectoryPlotter plotter) {
      plotter.plotTrajectory(m_trajectory1);
      plotter.plotTrajectory(m_trajectory2);
      plotter.plotTrajectory(m_trajectory3);
      plotter.plotTrajectory(m_trajectory4);
  }
}
