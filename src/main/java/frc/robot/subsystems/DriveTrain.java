package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

    public DriveTrain() {
        
        m_rightMotors.setInverted(true);
        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_differentialDrive.setSafetyEnabled(false);
    }

    public double getRightSpeed() {
        return -m_rightMotors.get();
    }

    public double getLeftSpeed() {
        return m_leftMotors.get();
    }

    public double getLeftEncoderDistance() {
        return m_leftEncoder.getDistance();
    }
    
    public double getRightEncoderDistance() {
        return m_rightEncoder.getDistance();
    }

    @Override
    public void periodic() {
    }

    public void drive(double throttle, double rotate, boolean squaredInput){
        m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInput);
    }
}