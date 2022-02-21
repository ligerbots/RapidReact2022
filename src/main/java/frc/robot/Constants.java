// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Temporary Encoder's Distance Per Pulse for simulation
    public static final double ENCODER_DISTANCE_PER_PULSE = Units.inchesToMeters(Math.PI * 6.0 / 1024);

    //Temporary variable values for simulation
    public static final double kvVoltSecondsPerMeter = 2.64; 
    public static final double kaVoltSecondsSquaredPerMeter = 0.324; 
    public static final double kvVoltSecondsPerRadian = 3.0;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
    LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

    public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);
    public static final double kDriveGearing = 8;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kTrackwidth = 0.6604;
    // more fake values for ramsete
    public static final double ksVolts = 0.182; 
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);
    public static final double kMaxSpeed = 1.75;
    public static final double kMaxAcceleration = 1.5;
    public static final double kPDriveVel = 6; 

    public static final double kRamseteB = 2; // generic ramsete values
    public static final double kRamseteZeta = 0.7; // generic ramsete values

    // Following four CAN IDs are for the drivetrain subsystem
    public static final int LEADER_LEFT_CAN_ID = 9; 
    public static final int LEADER_RIGHT_CAN_ID = 11;
    public static final int FOLLOWER_LEFT_CAN_ID = 12;
    public static final int FOLLOWER_RIGHT_CAN_ID = 8;

    // Following four CAN IDs are for the climber subsystem
    public static final int ELEVATOR_LEADER_CAN_ID = 1;
    public static final int ELEVATOR_FOLLOWER_CAN_ID = 2;
    public static final int ARM_LEADER_CAN_ID = 7;
    public static final int ARM_FOLLOWER_CAN_ID = 10;

    // intake subsystem
    public static final int INTAKE_MOTOR_CAN_ID = 5; 
    public static final double INTAKE_SHOOTING_SPEED = 5.0;

    // drivetrain encoders
    public static final int[] LEFT_ENCODER_PORTS = new int[]{0, 1};
    public static final int[] RIGHT_ENCODER_PORTS = new int[]{2, 3};
    public static final int LIMIT_SWITCH_ONE = 4; // Limit Switch 1
    public static final int LIMIT_SWITCH_TWO = 5; // Limit Switch 2

    // Following two CAN IDs are for the shooter subsystem
    public static final int TOP_SHOOTER_CAN_ID = 3; 
    public static final int BOTTOM_SHOOTER_CAN_ID = 4;

    // chute subsystem
    public static final int CHUTE_CAN_ID = 6; 

    public static final int GRAYHILL_ENCODER_DISTANCE_PER_PULSE = 1; //TODO: Replace this value with a correct one 

    public static final int XBOX_A = 1;

    // Vision Subsystem
    public static final double TOLERANCE_DEG = 5.;
    public static final double STABALIZE_SECS = .5;
    public static final double TIMEOUT_SECS = 3;
}