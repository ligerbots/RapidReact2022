// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int LIMIT_SWITCH_ONE = 4; // Limit Switch 1
    public static final int LIMIT_SWITCH_TWO = 5; // Limit Switch 2

    public static final int SHOOTER_ONE_CAN_ID = 10; // Motor 1 on shooter
    public static final int SHOOTER_TWO_CAN_ID = 11; // Motor 2 on shooter

    public static final int HOPPER_ONE_CAN_ID = 12; // Motor 1 on hopper
    public static final int HOPPER_TWO_CAN_ID = 13; // Motor 2 on hopper

    public static final int GRAYHILL_ENCODER_DISTANCE_PER_PULSE = 1; //TODO: Replace this value with a correct one 

    

}
