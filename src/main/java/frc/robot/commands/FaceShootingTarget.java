/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class FaceShootingTarget extends CommandBase {
  /**
   * Creates a new FaceShootingTarget.
   */
  // double startingAngle;
  DriveTrain robotDrive;
  DriveCommand driveCommand;
  Vision vision;
  double acceptableError;

  boolean oldOldCheck;
  boolean oldCheck;
  boolean check;

  boolean targetAcquired;
  double headingError;
  double headingTarget;

  double startTime;

  public FaceShootingTarget(DriveTrain robotDrive, double acceptableError, DriveCommand driveCommand, Vision vision) {
    this.robotDrive = robotDrive;
    this.acceptableError = acceptableError;
    this.driveCommand = driveCommand;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (driveCommand != null) driveCommand.cancel();

    vision.setMode(Vision.VisionMode.HUBFINDER);

    //shooter.setTurretAdjusted(Constants.TURRET_ANGLE_ZERO_SETTING);
    targetAcquired = false;
    oldOldCheck = false;
    check = false;
    oldCheck = false;
    startTime = Timer.getFPGATimestamp();
    //headingError = 100000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (targetAcquired) {
      headingError = robotDrive.getHeading() - headingTarget;
      while ( headingError > 180.0) headingError -= 360.0;
      while ( headingError < -180.0) headingError += 360.0;
      SmartDashboard.putNumber("shooter/HeadingError", headingError);
      System.out.println("FaceShooter headingError = " + headingError);

      check = Math.abs(headingError) < acceptableError && oldCheck;
      // System.out.format("FaceShootingTarget: %3.2f%n", initialAngleOffset);
      robotDrive.drive(0, robotDrive.turnSpeedCalc(headingError), false);

      oldCheck = Math.abs(headingError) < acceptableError && oldOldCheck;

      oldOldCheck = Math.abs(headingError) < acceptableError;
    }
    else if (vision.getStatus() && vision.getDistance() > 1.0)
    {
      double startAngle = robotDrive.getHeading();
      double visionAngle = vision.getRobotAngle();
      headingTarget = startAngle - visionAngle;
      System.out.format("FaceShooter acquired: heading = %3.1f visionAngle = %3.1f targetHeading = %3.2f%n",
                        startAngle, visionAngle, headingTarget);
      targetAcquired = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.format("FaceShooter finished: currentHeading = %3.1f targetHeading = %3.2f%n",
        robotDrive.getHeading(), headingTarget);

    robotDrive.tankDriveVolts(0, 0);
    if (driveCommand != null) driveCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (Math.abs(headingError) < acceptableError && check) 
        || (!targetAcquired && (Timer.getFPGATimestamp() - startTime) > 0.5) 
        || Timer.getFPGATimestamp() - startTime > 3.0;
  }
}