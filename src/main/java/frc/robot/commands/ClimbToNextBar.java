package frc.robot.commands;

import java.util.zip.Deflater;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimbToNextBar extends CommandBase {

    enum STAGE {
        STAGE1, WAIT_STAGE1, STAGE2, WAIT_STAGE2, STAGE3, WAIT_STAGE3, STAGE4, WAIT_STAGE4, STAGE5, WAIT_STAGE5, STAGE6,
        WAIT_STAGE6, FINISHED;
    }

    Climber m_climber;
    STAGE m_stage;

    public ClimbToNextBar(Climber climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_stage = STAGE.STAGE1;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (m_stage) {
            case STAGE1:
                // rotate the arm to point elevator towards the next bar
                m_climber.setArmAngle(Constants.ARM_ANGLE_TO_NEXT_BAR);
                m_stage = STAGE.WAIT_STAGE1;
            case WAIT_STAGE1:
                if (Math.abs(m_climber.getArmAngle() - Constants.ARM_ANGLE_TO_NEXT_BAR) < Constants.ARM_ANGLE_TOLERANCE)
                    m_stage = STAGE.STAGE2;
                break;
            case STAGE2:
                // extend the elevator
                m_climber.setElevatorHeight(Constants.ELEVATOR_MAX_HEIGHT);
                m_stage = STAGE.WAIT_STAGE2;
            case WAIT_STAGE2:
                if (Math.abs(m_climber.getElevatorHeight()
                        - Constants.ELEVATOR_MAX_HEIGHT) < Constants.ELEVATOR_HEIGHT_TOLERANCE)
                    m_stage = STAGE.STAGE3;
                break;

            case STAGE3:
                // parallel command group, to have the arm rotate while elevator retracts
                new RetractElevatorAdjustArm(m_climber).schedule();
                m_stage = STAGE.WAIT_STAGE3;
            case WAIT_STAGE3:
                if (Math.abs(m_climber.getArmAngle() - Constants.ARM_ADJUST_ANGLE) < Constants.ARM_ANGLE_TOLERANCE
                        && Math.abs(m_climber.getElevatorHeight()
                                - (Constants.ELEVATOR_MIN_HEIGHT + 20.0)) < Constants.ELEVATOR_HEIGHT_TOLERANCE)
                    m_stage = STAGE.STAGE4;
                break;
            case STAGE4:
                // rotate the arm to leave the previous bar and get to the side of the next bar
                m_climber.setArmAngle(45.0);
                m_stage = STAGE.WAIT_STAGE4;
            case WAIT_STAGE4:
                if (Math.abs(m_climber.getArmAngle() - 45.0) < Constants.ARM_ANGLE_TOLERANCE)
                    m_stage = STAGE.STAGE5;
                break;
            case STAGE5:
                // fully retract the elevator
                m_climber.setElevatorHeight(Constants.ELEVATOR_MIN_HEIGHT);
                m_stage = STAGE.WAIT_STAGE5;
            case WAIT_STAGE5:
                if (Math.abs(m_climber.getElevatorHeight()
                        - Constants.ELEVATOR_MIN_HEIGHT) < Constants.ELEVATOR_HEIGHT_TOLERANCE)
                    m_stage = STAGE.STAGE6;
                break;
            case STAGE6:
                // rotate the arm back to grab on to the bar
                m_climber.setArmAngle(90.0);
                m_stage = STAGE.WAIT_STAGE6;
            case WAIT_STAGE6:
                if (Math.abs(m_climber.getArmAngle() - 90.0) < Constants.ARM_ANGLE_TOLERANCE)
                    m_stage = STAGE.FINISHED;
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_stage == STAGE.FINISHED;
    }

}