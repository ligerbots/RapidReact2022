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

    public static final double FALCON_UNITS_PER_RPM = 2048.0 / 600.0;

    // Temporary variable values for simulation
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
    public static final int[] ELEVATOR_CAN_IDS = new int[] {1,2};
    public static final int[] ARM_CAN_IDS = new int[] {7,10};
    
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

    // chute subsystem
    public static final int CHUTE_CAN_ID = 6; 

    public static final int GRAYHILL_ENCODER_DISTANCE_PER_PULSE = 1; //TODO: Replace this value with a correct one 

    //define constants for high, low, and mid rung
    public static final int HIGH_RUNG = 192;//192 cm
    public static final int MID_RUNG = 153;//153 cm
    public static final int LOW_RUNG = 124;//Top of rung is 124cm
    public static final double CLIMBER_ANGLE = 22.0;//angle for setClimber()
    
    
    public static final double ELEVATOR_HEIGHT_TOLERANCE = 0.5;//tolerance for elevator
    public static final double ARM_ANGLE_TOLERANCE = 0.5;

    public static final double RUNG_ANGLE = -22.0;//angle to clamp back on rung for raiseToBar command
    
    public static final double ELEVATOR_MAX_HEIGHT = 100.0;// 23.5in = length of elevator when fully extended
    public static final double ELEVATOR_MIN_HEIGHT = -100.0;// 0in = length of elevator when fully retracted
    // right elevator 0 to -21.5
     
    // Feedforward constants for the each Climber Arm
    public static final double ARM_KS = 0.182; // TODO: This may need to be tuned
    // The following constants are computed from https://www.reca.lc/arm
    public static final double ARM_KG = 2.17;
    public static final double ARM_KV = 1.8;
    public static final double ARM_KA = 0.07;

    // Constants to limit the arm rotation speed
    public static final double ARM_MAX_VEL_RAD_PER_SEC = Math.toRadians(30.0);
    public static final double ARM_MAX_ACC_RAD_PER_SEC_SQ = Math.toRadians(30.0);
    public static final double ARM_OFFSET_RAD = Math.toRadians(80.0);

    // PID Constants for the Arm PID controller
    // Since we're using Trapeziodal control, all values will be 0 except for P
    public static final double ARM_K_P = 5.0e-5;
    public static final double ARM_K_I = 5.0e-5;
    public static final double ARM_K_D = 5.0e-5;
    public static final double ARM_K_FF = 5.0e-5;

    // Limit the arm rotation
    // TODO: This is relative to 0 starting position. Need to use absolute encoder and get better values
    public static final double ARM_MAX_ANGLE = 20.0;
    public static final double ARM_MIN_ANGLE = -20.0;

    //the angle for the arm to rotate to turn the elevator towards the next bar
    public static final double ARM_ANGLE_TO_NEXT_BAR = 130.0;
    public static final double ARM_ADJUST_ANGLE = 160.0; // the angle the arm needs to rotate to follow the motion of the elevator when retracting

    //the angle for the arm to rotate to the left side of the next bar
    public static final double ARM_TO_THE_LEFT_ANGLE = 45.0;
    public static final double ARM_GRAB_THE_BAR = 90.0;

    //the height of the elevator to retract down to certain point where the arm can get to the other side of the bar
    public static final double ELEVATOR_HEIGHT_FOR_ARM_CLEARANCE = ELEVATOR_MIN_HEIGHT + 20.0;
    public static final double ARM_ANGLE_FOR_ELEVATOR_CLEARANCE = ARM_GRAB_THE_BAR - 10.0;

    // drivetrain turning constants
    public static final double TURN_TOLERANCE_DEG = 5.;
    public static final double TURN_STABILIZE_SECS = .5;
    public static final double TURN_TIMEOUT_SECS = 3;
    
    //Xbox button mapping
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