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

public class TwoBallAutoCurved extends SequentialCommandGroup implements AutoCommandInterface {
    Trajectory m_trajectory;
    public TwoBallAutoCurved(Shooter shooter, Intake intake, DriveTrain driveTrain, Vision vision) {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

        TrajectoryConfig config =
            new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAcceleration)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);

        m_trajectory = TrajectoryGenerator.generateTrajectory(
            getInitialPose(), 
            List.of(),
            FieldInformation.middleBlueBall,
            config
        ); 

        RamseteCommand ramseteCommand = new RamseteCommand(
            m_trajectory,
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
                ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new IntakeCommand(intake, Constants.INTAKE_SPEED)
            ),
            new FaceShootingTarget(driveTrain, vision, Constants.TURN_TOLERANCE_DEG, null),
            new ShooterCommand(shooter, intake, vision, true)
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldInformation.middleBlueStart;
    }
    
    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.plotTrajectory(m_trajectory);
    }
}
