// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendElevatorTest extends SequentialCommandGroup {
  /** Creates a new ExtendElevatorTest. */
  public ExtendElevatorTest(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorHeight(climber, Constants.ELEVATOR_MAX_HEIGHT, 
          Constants.ELEVATOR_MAX_VEL_METER_PER_SEC_ASCEND, Constants.ELEVATOR_MAX_ACC_METER_PER_SEC_SQ_ASCEND),

      // Now coast the elevators until they go all the way up
      new CoastExtendElevator(climber)
    );
  }
}
