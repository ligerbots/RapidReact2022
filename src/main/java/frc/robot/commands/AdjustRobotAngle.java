// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AdjustRobotAngle extends TrapezoidProfileCommand {
  /** Creates a new AdjustRobotAngle. */
  public AdjustRobotAngle(DriveTrain driveTrain, double goalAngle) {
    super(
        // The motion profile to be executed
        new TrapezoidProfile(
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0),
            // Goal state

            //TODO: do we want to use radians?
            new TrapezoidProfile.State(),
            // Initial state
            new TrapezoidProfile.State()),
        state -> {});
  }
}
