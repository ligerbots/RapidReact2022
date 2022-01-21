package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI.Port;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    private DifferentialDrivetrainSim m_differentialDriveSim;
    private EncoderSim m_leftEncoderSim;
    private EncoderSim m_rightEncoderSim;
    private Field2d m_fieldSim;
    private SimDouble m_gyroAngleSim;

    private DifferentialDriveOdometry m_odometry;

    private AHRS m_navX;

    public DriveTrain() {
        
        m_rightMotors.setInverted(true);
        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_differentialDrive.setSafetyEnabled(false);

        m_leftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        m_rightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);

        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
        m_navX = new AHRS(Port.kMXP, (byte) 200);


        if (RobotBase.isSimulation()) {

            m_differentialDriveSim = new DifferentialDrivetrainSim(
                Constants.kDrivetrainPlant,
                Constants.kDriveGearbox,
                Constants.kDriveGearing,
                Constants.kTrackwidth,
                Constants.kWheelDiameterMeters / 2.0,
                null);

            m_leftEncoderSim = new EncoderSim(m_leftEncoder);
            m_rightEncoderSim = new EncoderSim(m_rightEncoder);

            m_gyroAngleSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");
            
            m_fieldSim = new Field2d();
            SmartDashboard.putData("Field", m_fieldSim);
        }
    }

    //Get the current set speed of the speed controllers
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
        return m_odometry.getPoseMeters();
    }

    public void setPose (Pose2d pose){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
    }

    public double getHeading() {
        return m_odometry.getPoseMeters().getRotation().getDegrees();
    }


    //Get Gyro info
    public double getGyroAngle() {
        return Math.IEEEremainder(m_navX.getAngle(), 360) * -1;
    }


    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getGyroAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

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

    @Override
    public void simulationPeriodic() {
        // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        m_differentialDriveSim.setInputs(m_leftMotors.get() * RobotController.getInputVoltage(),
                             m_rightMotors.get() * RobotController.getInputVoltage());
      
        // Advance the model by 20 ms. Note that if you are running this
        // subsystem in a separate thread or have changed the nominal timestep
        // of TimedRobot, this value needs to match it.
        m_differentialDriveSim.update(0.02);
      
        // Update all of our sensors.
        m_leftEncoderSim.setDistance(m_differentialDriveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_differentialDriveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_differentialDriveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_differentialDriveSim.getRightVelocityMetersPerSecond());
        m_gyroAngleSim.set(-m_differentialDriveSim.getHeading().getDegrees());
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }


}
