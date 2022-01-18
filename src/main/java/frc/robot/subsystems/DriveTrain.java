package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    private WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.LEADER_LEFT_CAN_ID);
    private WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.FOLLOWER_LEFT_CAN_ID);
    private WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.LEADER_RIGHT_CAN_ID);
    private WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.FOLLOWER_RIGHT_CAN_ID);

    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);

    private DifferentialDrive m_differentialDrive;

    private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
    private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

    private DifferentialDriveOdometry odometry;

    private AHRS navX;

    public DriveTrain() {
        
        m_rightMotors.setInverted(true);
        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_differentialDrive.setSafetyEnabled(false);

        m_leftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        m_rightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
        navX = new AHRS(Port.kMXP, (byte) 200);
    }

    //Get the speed of the motors
    public double getRightSpeed() {  
        return -m_rightMotors.get();
    }

    public double getLeftSpeed() {
        return m_leftMotors.get();
    }

    //Get stats about the encoders
    public double getLeftEncoderDistance() {
        return m_leftEncoder.getDistance();
    }
    
    public double getRightEncoderDistance() {
        return m_rightEncoder.getDistance();
    }
    
    public int getLeftEncoderTicks(){
        return m_leftEncoder.get();
    }

    public int getRightEncoderTicks(){
        return m_rightEncoder.get();
    }

    //Get and Set odometry values
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public void setPose (Pose2d pose){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
    }

    public double getHeading() {
        return odometry.getPoseMeters().getRotation().getDegrees();
    }


    //Get Gyro info
    public double getGyroAngle() {
        return Math.IEEEremainder(navX.getAngle(), 360) * -1;
    }


    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getGyroAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

        SmartDashboard.putNumber("driveTrain/heading", getHeading());
        SmartDashboard.putNumber("driveTrain/NavX gyro", getGyroAngle());
        SmartDashboard.putNumber("driveTrain/x position", getPose().getX());
        SmartDashboard.putNumber("driveTrain/y position", getPose().getY());

        SmartDashboard.putNumber("driveTrain/left encoder", getLeftEncoderTicks());
        SmartDashboard.putNumber("driveTrain/right encoder", getRightEncoderTicks());
    }

    public void drive(double throttle, double rotate, boolean squaredInput){
        m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInput);
    }


}