package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI.Port;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {

    private WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.LEADER_LEFT_CAN_ID);
    private WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.FOLLOWER_LEFT_CAN_ID);
    private WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.LEADER_RIGHT_CAN_ID);
    private WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.FOLLOWER_RIGHT_CAN_ID);

    private TalonFXSimCollection m_leftLeader_sim;
    private TalonFXSimCollection m_rightLeader_sim;
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);

    private DifferentialDrive m_differentialDrive;

    // private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
    // private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

    private DifferentialDrivetrainSim m_differentialDriveSim;
    // private EncoderSim m_leftEncoderSim;
    // private EncoderSim m_rightEncoderSim;
    private Field2d m_fieldSim;
    private SimDouble m_gyroAngleSim;
    
    private DifferentialDriveOdometry m_odometry;

    private AHRS m_navX;

    public DriveTrain() {
        // setup PID control for TalonFX
        m_leftLeader.configFactoryDefault();
        m_leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_leftLeader.set(ControlMode.Position,0);
        m_leftLeader.config_kP(0, Constants.DRIVETRAIN_KP);
        m_leftLeader.config_kI(0, Constants.DRIVETRAIN_KI);
        m_leftLeader.config_kD(0, Constants.DRIVETRAIN_KD);
        m_leftLeader.config_kF(0, Constants.DRIVETRAIN_KF);
        m_leftLeader.setSensorPhase(false);

        m_rightLeader.configFactoryDefault();
        m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_rightLeader.set(ControlMode.Position,0);
        m_rightLeader.config_kP(0, Constants.DRIVETRAIN_KP);
        m_rightLeader.config_kI(0, Constants.DRIVETRAIN_KI);
        m_rightLeader.config_kD(0, Constants.DRIVETRAIN_KD);
        m_rightLeader.config_kF(0, Constants.DRIVETRAIN_KF);
        m_rightLeader.setSensorPhase(true);
        
        m_rightMotors.setInverted(true);
        setMotorMode(NeutralMode.Coast);
    
       
        m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_differentialDrive.setSafetyEnabled(false);

        // m_leftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        // m_rightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        // m_rightEncoder.setReverseDirection(true);
        
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

            m_leftLeader_sim = m_leftLeader.getSimCollection();
            m_rightLeader_sim = m_rightLeader.getSimCollection();
                    // m_leftEncoderSim = new EncoderSim(m_leftEncoder);
            // m_rightEncoderSim = new EncoderSim(m_rightEncoder);
            m_gyroAngleSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");

            m_fieldSim = new Field2d();
            SmartDashboard.putData("Field", m_fieldSim);
        }
    }

    public void setMotorMode(NeutralMode m) {
        m_leftLeader.setNeutralMode(m);
        m_leftFollower.setNeutralMode(m);
        m_rightLeader.setNeutralMode(m);
        m_rightFollower.setNeutralMode(m); 
    }

    // Get the current set speed of the speed controllers
    public double getRightSpeed() {
        return -m_rightMotors.get();
    }

    public double getLeftSpeed() {
        return m_leftMotors.get();
    }

    // Get stats about the encoders
    public double getLeftEncoderDistance() {
        // return m_leftEncoder.getDistance();
        return m_leftLeader.getSelectedSensorPosition() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT;
    }
    public void setLeftEncoderDistance(double distance) {
        m_leftLeader.setSelectedSensorPosition((int) (distance / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    }

    public double getRightEncoderDistance() {
        // return m_rightEncoder.getDistance();
        return -m_rightLeader.getSelectedSensorPosition() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT;
    }
    public void setRightEncoderDistance(double distance) {
        m_rightLeader.setSelectedSensorPosition((int) (-distance / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    }
    public double getDistance() {
        return 0.5 * (getLeftEncoderDistance() + getRightEncoderDistance());
    }

    public double getLeftEncoderVelocity() {
        // sensor velocity is per 100ms, so an extra scale of 10
        return m_leftLeader.getSelectedSensorVelocity() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT * 10.0;
    }
    public double getRightEncoderVelocity() {
        // sensor velocity is per 100ms, so an extra scale of 10
        return -m_rightLeader.getSelectedSensorVelocity() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT * 10.0;
    }

    public int getLeftEncoderTicks() {
        // return m_leftEncoder.get();
        return (int)m_leftLeader.getSelectedSensorPosition();
    }
    public void setLeftEncoderTicks(int ticks){
        m_leftLeader.setSelectedSensorPosition(ticks);
    }
    public int getRightEncoderTicks() {
        // return m_rightEncoder.get();
        return (int)m_rightLeader.getSelectedSensorPosition();
    }
    public void setRightEncoderTicks(int ticks){
        m_rightLeader.setSelectedSensorPosition(ticks);
    }
    // Get and Set odometry values
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        // m_leftEncoder.reset();
        // m_rightEncoder.reset();
        m_leftLeader.setSelectedSensorPosition(0.0);
        m_rightLeader.setSelectedSensorPosition(0.0);

        if (Robot.isSimulation()) m_differentialDriveSim.setPose(new Pose2d()); // drive sim doesn't seem to get reset anymore?
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
    }

    public double getHeading() {
        return m_odometry.getPoseMeters().getRotation().getDegrees();
    }

    // Get Gyro info
    public double getGyroAngle() {
        return Math.IEEEremainder(m_navX.getAngle(), 360) * -1;
    }

    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftEncoderDistance(),
                getRightEncoderDistance());

        SmartDashboard.putNumber("driveTrain/heading", getHeading());
        SmartDashboard.putNumber("driveTrain/NavX gyro", getGyroAngle());
        SmartDashboard.putNumber("driveTrain/x position", getPose().getX());
        SmartDashboard.putNumber("driveTrain/y position", getPose().getY());

        SmartDashboard.putNumber("driveTrain/left encoder", getLeftEncoderTicks());
        SmartDashboard.putNumber("driveTrain/right encoder", getRightEncoderTicks());
        SmartDashboard.putNumber("driveTrain/left distance", getLeftEncoderDistance());
        SmartDashboard.putNumber("driveTrain/right distance", getRightEncoderDistance());

        // SmartDashboard.putNumber("driveTrain/LeftFollower", m_leftFollower.getSelectedSensorPosition());
        // SmartDashboard.putNumber("driveTrain/LeftLeader", m_leftLeader.getSelectedSensorPosition());
        // SmartDashboard.putNumber("driveTrain/RightFollower", m_rightFollower.getSelectedSensorPosition());
        // SmartDashboard.putNumber("driveTrain/RightLeader", m_rightLeader.getSelectedSensorPosition());
    }

    public void drive(double throttle, double rotate, boolean squaredInput) {
        // SmartDashboard.putNumber("driveTrain/throttle", throttle);

        m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInput);
    }

    public void tankDriveVolts (double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_differentialDrive.feed();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
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
        /*
        m_leftEncoderSim.setDistance(m_differentialDriveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_differentialDriveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_differentialDriveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_differentialDriveSim.getRightVelocityMetersPerSecond());
        */

        m_leftLeader_sim.setIntegratedSensorRawPosition((int) (m_differentialDriveSim.getLeftPositionMeters() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
        m_leftLeader_sim.setIntegratedSensorVelocity((int) (m_differentialDriveSim.getLeftVelocityMetersPerSecond() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT / 10.0));

        m_rightLeader_sim.setIntegratedSensorRawPosition( - (int) (m_differentialDriveSim.getRightPositionMeters() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
        m_rightLeader_sim.setIntegratedSensorVelocity( - (int) (m_differentialDriveSim.getRightVelocityMetersPerSecond() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT / 10.0));

        m_gyroAngleSim.set(-m_differentialDriveSim.getHeading().getDegrees());
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }

    public Field2d getField2d() {
        return m_fieldSim;
    }

    public double disToTurn(double angle){
        // calculate the distance one side of the wheels need to turn to get to the desired angle
        return Constants.kTrackwidth*Math.PI*(angle / (2*Math.PI));
    }

    public void setSetPoint(boolean turnToLeft, TrapezoidProfile.State setPoint, double startDisLeft, double startDisRight) {
        // if turn to left, let the right motor drive, and vice versa
        if(turnToLeft){
            // have the left side go forward and right side backward to turn the robot to the right
            m_leftLeader.set(ControlMode.Position, startDisLeft + setPoint.position);
            m_rightLeader.set(ControlMode.Position, startDisRight - setPoint.position);

            SmartDashboard.putNumber("DriveTrain/setPointLeft", Units.metersToInches(startDisLeft + setPoint.position));
            SmartDashboard.putNumber("DriveTrain/setPointRight", Units.metersToInches(startDisRight - setPoint.position));
        }else{
            // have the right side go forward and left side backward to turn the robot to the left
            m_leftLeader.set(ControlMode.Position, startDisLeft - setPoint.position);
            m_rightLeader.set(ControlMode.Position, startDisRight + setPoint.position);

            SmartDashboard.putNumber("DriveTrain/setPointLeft", Units.metersToInches(startDisLeft - setPoint.position));
            SmartDashboard.putNumber("DriveTrain/setPointRight", Units.metersToInches(startDisRight + setPoint.position));
        }
        SmartDashboard.putBoolean("DriveTrain/turnToLeft", turnToLeft);        
    }

    public double turnSpeedCalc(double angleError) {
        double absErr = Math.abs(angleError);
        double turnSpeed;
        if (absErr > 60.0) {
            turnSpeed = 0.6;//0.8;
        }
        else if (absErr > 30.0) {
            turnSpeed = 0.15;//0.2; //0.4;
        }
        else if (absErr > 10.0) {
            turnSpeed = 0.1;//0.15;
        }
        // else if (absErr > 5.0) {
        //     turnSpeed = 0.1; //0.07;
        // }
        else {
            turnSpeed = 0.125; // 0.065;
        }

        return turnSpeed * Math.signum(angleError);
    }
}
