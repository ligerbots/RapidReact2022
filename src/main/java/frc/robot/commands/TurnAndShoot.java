package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class TurnAndShoot extends SequentialCommandGroup {

    public TurnAndShoot(Shooter shooter, Intake intake, DriveTrain driveTrain, Vision vision, DriveCommand driveCommand) {
        addCommands(
            // new TurnTowardsHub(driveTrain, vision).withTimeout(Constants.TURN_TIMEOUT_SECS),
            new FaceShootingTarget(driveTrain, vision, Constants.TURN_TOLERANCE_DEG, driveCommand),
            new ShooterCommand(shooter, intake, vision, true)
        );
    }
}