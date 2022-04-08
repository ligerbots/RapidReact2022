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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.FieldInformation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallMiddle extends SequentialCommandGroup implements AutoCommandInterface {
  /** Creates a new ThreeBallLower. */

    static final double DISTANCE_BACK = 1.4;
    Trajectory m_initialTrajectory;
    Trajectory m_finalTrajectory;
    public ThreeBallMiddle(Shooter shooter, Intake intake, DriveTrain driveTrain, Vision vision) {
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
        Pose2d cornerPose = FieldInformation.ballPosePolar(FieldInformation.cornerBlueBall, 11, 20);
        Pose2d finalPose = FieldInformation.middleBlueBall;
        // Pose2d midPose = new Pose2d(
        //     initialPose.getX() - initialPose.getRotation().getCos() * DISTANCE_BACK, 
        //     initialPose.getY() - initialPose.getRotation().getSin() * DISTANCE_BACK, 
        //     initialPose.getRotation());

        m_initialTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPose, 
            List.of(
                FieldInformation.middleBlueBall.getTranslation()
            ),
            cornerPose,
            reverseConfig
        );
        
        m_finalTrajectory = TrajectoryGenerator.generateTrajectory(
            cornerPose, 
            List.of(),
            finalPose,
            forwardConfig
        );

        RamseteCommand ramsete1 = new RamseteCommand(
            m_initialTrajectory,
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
            m_finalTrajectory,
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
            new ShooterCommand(shooter, intake, Constants.STARTING_DISTANCE, true),
            new ParallelDeadlineGroup(
                ramsete1.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new IntakeCommand(intake, Constants.INTAKE_SPEED)
            ),
            new ParallelDeadlineGroup(
                ramsete2.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new IntakeCommand(intake, Constants.INTAKE_SPEED)
            ),
            new TurnAndShoot(shooter, intake, driveTrain, vision, null)
        );
    }
  
  @Override
  public Pose2d getInitialPose() {
      return FieldInformation.middleBlueStart;
  }
  
  @Override
  public void plotTrajectory(TrajectoryPlotter plotter) {
      // with multiple trajectories, you need to select an index for each
      plotter.plotTrajectory(0, m_initialTrajectory);
      plotter.plotTrajectory(1, m_finalTrajectory);
  }
}
