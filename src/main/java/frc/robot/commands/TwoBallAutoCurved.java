package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldInformation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TwoBallAutoCurved extends SequentialCommandGroup implements AutoCommandInterface {
    static final double DISTANCE_BACK = 1.4;
    Trajectory trajectory;
    public TwoBallAutoCurved(Shooter shooter, Intake intake, DriveTrain driveTrain, Vision vision) {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

        TrajectoryConfig config =
            new TrajectoryConfig(Constants.kMaxSpeed,
                                Constants.kMaxAcceleration)
                .setKinematics(Constants.kDriveKinematics)
                .addConstraint(autoVoltageConstraint)
                .setReversed(true);

        trajectory = TrajectoryGenerator.generateTrajectory(
            getInitialPose(), 
            List.of(),
            FieldInformation.middleBlueBall,
            config
        ); 

        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
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
            new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                    ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                    new WaitCommand(0.5) // for the ball to enter the hopper
                ), 
                new IntakeCommand(intake, Constants.INTAKE_SHOOTING_SPEED)
            ),
            new TurnTowardsHub(driveTrain, vision).withTimeout(Constants.TIMEOUT_SECS),
            new SelectCommand(() -> new ShooterCommand(shooter, intake, vision, Shooter.calculateShooterSpeeds(vision.getDistance())))
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldInformation.middleBlueStart;
    }
    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        plotter.plotTrajectory(trajectory);
    }
}
