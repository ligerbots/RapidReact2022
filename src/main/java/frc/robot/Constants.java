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
    // Temporary variable values for simulation
    public static final double kvVoltSecondsPerMeter = 2.64; 
    public static final double kaVoltSecondsSquaredPerMeter = 0.324; 
    public static final double kvVoltSecondsPerRadian = 3.0;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
    LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
    public static final double kDriveGearing = 10.125;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6.0);   // 6 inch wheels
    public static final double kTrackwidth = 0.6604;
    // more fake values for ramsete
    public static final double ksVolts = 0.182; 
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);
    public static final double kMaxSpeed = 1.75;
    public static final double kMaxAcceleration = 1.5;
    public static final double kPDriveVel = 6; 

    public static final double kRamseteB = 2; // generic ramsete values
    public static final double kRamseteZeta = 0.7; // generic ramsete values

    // Grayhill is 256 ticks per revolution. Wheel diameter is 6".
    public static final double ENCODER_DISTANCE_PER_PULSE = Units.inchesToMeters(Math.PI * 6.0 / 256);

    public static final double FALCON_UNITS_PER_RPM = 2048.0 / 600.0;
    public static final double DRIVE_FALCON_DISTANCE_PER_UNIT = Units.inchesToMeters(Math.PI * 6.0) / kDriveGearing / 2048.0;

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
    public static final double INTAKE_SHOOTING_SPEED = 0.4;
    public static final double INTAKE_SPEED = 0.4;

    // drivetrain encoders
    public static final int[] LEFT_ENCODER_PORTS = new int[]{0, 1};
    public static final int[] RIGHT_ENCODER_PORTS = new int[]{2, 3};
    public static final int LIMIT_SWITCH_ONE = 4; // Limit Switch 1
    public static final int LIMIT_SWITCH_TWO = 5; // Limit Switch 2

    // Following two CAN IDs are for the shooter subsystem
    public static final int TOP_SHOOTER_CAN_ID = 3; 
    public static final int BOTTOM_SHOOTER_CAN_ID = 4;
    public static final double SHOOTER_KP = 0.03;     // CTRE example = 0.1
    public static final double SHOOTER_KI = 0.0;   // CTRE example = 0.001
    public static final double SHOOTER_KD = 0.0;     // CTRE example = 5.0
    // From example code: kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
    public static final double SHOOTER_KF = 1023.0/20660.0;  // CTRE example = 1023.0/20660.0

    // amount of time to wait for the motors on shooters to speed up, later replaced with checking RPM
    public static final double SHOOTER_MOTOR_WAIT_TIME = 0.5;
    // time to wait for the shots from the shooter
    public static final double SHOOTER_INTAKE_WAIT_TIME = 0.25;
    // amount of times to shoot the first ball
    public static final double SHOOT_BALL1_WAIT_TIME = 0.5;
    // amount of time to shoot the second ball
    public static final double SHOOT_BALL2_WAIT_TIME = 0.5;
    

    // chute subsystem
    public static final int CHUTE_CAN_ID = 6; 

    // define constants for high, low, and mid rung
    public static final int HIGH_RUNG = 192;//192 cm
    public static final int MID_RUNG = 153;//153 cm
    public static final int LOW_RUNG = 124;//Top of rung is 124cm
    public static final double CLIMBER_ANGLE = 22.0;//angle for setClimber()
    
    
    public static final double ELEVATOR_HEIGHT_TOLERANCE = 0.5;//tolerance for elevator
    public static final double ARM_ANGLE_TOLERANCE = 0.5;

    public static final double RUNG_ANGLE = -22.0;//angle to clamp back on rung for raiseToBar command
    
    public static final double ELEVATOR_MAX_HEIGHT = 200.0;//length of elevator when fully extended
    public static final double ELEVATOR_MIN_HEIGHT = 100.0;//length of elevator when fully retracted

    // the angle for the arm to rotate to turn the elevator towards the next bar
    public static final double ARM_ANGLE_TO_NEXT_BAR = 130.0;
    public static final double ARM_ADJUST_ANGLE = 160.0; // the angle the arm needs to rotate to follow the motion of the elevator when retracting

    // the angle for the arm to rotate to the left side of the next bar
    public static final double ARM_TO_THE_LEFT_ANGLE = 45.0;
    public static final double ARM_GRAB_THE_BAR = 90.0;



    // the height of the elevator to retract down to certain point where the arm can get to the other side of the bar
    public static final double ELEVATOR_HEIGHT_FOR_ARM_CLEARANCE = ELEVATOR_MIN_HEIGHT + 20.0;
    public static final double ARM_ANGLE_FOR_ELEVATOR_CLEARANCE = ARM_GRAB_THE_BAR - 10.0;

    // drivetrain turning constants
    public static final double TURN_TOLERANCE_DEG = 5.;
    public static final double TURN_STABILIZE_SECS = .5;
    public static final double TURN_TIMEOUT_SECS = 3;

    // Xbox button mapping
    public static final int XBOX_A = 1;
    public static final int XBOX_B = 2;
    public static final int XBOX_X = 3;
    public static final int XBOX_Y = 4;

    // bumpers
    public static final int XBOX_LB = 5;
    public static final int XBOX_RB = 6;
    
    public static final int XBOX_BACK = 7;
    public static final int XBOX_START = 8;

    // joy stick button
    public static final int XBOX_JL = 9;
    public static final int XBOX_JR = 10;
}
