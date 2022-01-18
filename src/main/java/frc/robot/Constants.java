// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Following four CAN IDs are for the drivetrain
    public static final int LEADER_LEFT_CAN_ID = 1; 
    public static final int LEADER_RIGHT_CAN_ID = 4;
    public static final int FOLLOWER_LEFT_CAN_ID = 2;
    public static final int FOLLOWER_RIGHT_CAN_ID = 3;

    // Following four CAN IDs are for the climber subsystem
    public static final int ELEVATOR_LEADER_CAN_ID = 5;
    public static final int ELEVATOR_FOLLOWER_CAN_ID = 6;
    public static final int ARM_LEADER_CAN_ID = 7;
    public static final int ARM_FOLLOWER_CAN_ID = 8;

    // Intake subsystem
    public static final int INTAKE_MOTOR_CAN_ID = 9; //temporary value

    // drivetrain encoders
    public static final int[] LEFT_ENCODER_PORTS = new int[]{0, 1};
    public static final int[] RIGHT_ENCODER_PORTS = new int[]{2, 3};
    public static final int LIMIT_SWITCH_ONE = 1; // Limit Switch 1
    public static final int LIMIT_SWITCH_TWO = 2; // Limit Switch 2

    public static final int SHOOTER_ONE_CAN_ID = 3; // Motor 1 on shooter
    public static final int SHOOTER_TWO_CAN_ID = 4; // Motor 2 on shooter

    public static final int HOPPER_ONE_CAN_ID = 5; // Motor 1 on hopper
    public static final int HOPPER_TWO_CAN_ID = 6; // Motor 2 on hopper

    public static final int ENCODER_DISTANCE_PER_PULSE = 1; //Temporary value

    

}
