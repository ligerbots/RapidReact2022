package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldInformation;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class OneBallAuto extends SequentialCommandGroup implements AutoCommandInterface {

    public OneBallAuto(Shooter shooter, Intake intake, DriveTrain driveTrain, Vision vision) {
        addCommands(
            new TurnTowardsHub(driveTrain, vision).withTimeout(Constants.TURN_TIMEOUT_SECS),
            new ShooterCommand(shooter, intake, vision, true, false)
        );
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldInformation.upperBlueStart;
    }
}
