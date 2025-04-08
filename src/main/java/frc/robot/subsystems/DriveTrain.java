package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
// import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {

    private TalonFX m_leftMotor;
    private TalonFX m_rightMotor;

    // private TalonFXSimCollection m_leftLeader_sim;
    // private TalonFXSimCollection m_rightLeader_sim;
    // private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
    // private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);

    private DifferentialDrive m_differentialDrive;

    // private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
    // private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

    // private DifferentialDrivetrainSim m_differentialDriveSim;
    // private EncoderSim m_leftEncoderSim;
    // private EncoderSim m_rightEncoderSim;
    // private Field2d m_fieldSim;
    // private SimDouble m_gyroAngleSim;
    
    // private DifferentialDriveOdometry m_odometry;

    // private AHRS m_navX;

    public DriveTrain() {
        m_leftMotor = new TalonFX(Constants.LEADER_LEFT_CAN_ID);
        m_rightMotor = new TalonFX(Constants.LEADER_RIGHT_CAN_ID);
    
        // set factory default
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        
        // set slot 0 gains
        Slot0Configs slot0configs = talonFXConfigs.Slot0;
        
        slot0configs.kP = Constants.DRIVETRAIN_KP;
        slot0configs.kI = Constants.DRIVETRAIN_KI;
        slot0configs.kD = Constants.DRIVETRAIN_KD;
        
        // m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        m_leftMotor.setPosition(0);

        // apply configs
        m_leftMotor.getConfigurator().apply(slot0configs);
        m_rightMotor.getConfigurator().apply(slot0configs);

        MotorOutputConfigs rightMotorConfigs = new MotorOutputConfigs();

        // set invert to CW+ and apply config change
        rightMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_leftMotor.getConfigurator().apply(rightMotorConfigs);

        // m_rightLeader.configFactoryDefault();
        // m_rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        // m_rightLeader.set(ControlMode.Position,0);
        // m_rightLeader.config_kP(0, Constants.DRIVETRAIN_KP);
        // m_rightLeader.config_kI(0, Constants.DRIVETRAIN_KI);
        // m_rightLeader.config_kD(0, Constants.DRIVETRAIN_KD);
        // m_rightLeader.config_kF(0, Constants.DRIVETRAIN_KF);
        // m_rightLeader.setSensorPhase(true);

        setMotorMode(NeutralModeValue.Coast);
        
        // m_differentialDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
        // m_differentialDrive.setSafetyEnabled(false);

        // // m_leftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        // // m_rightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
        // // m_rightEncoder.setReverseDirection(true);
        
        // m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);
        // m_navX = new AHRS(NavXComType.kMXP_SPI, (byte) 200);  // it's either kMXP_SPI or kMXP_UART

        // if (RobotBase.isSimulation()) {

        //     m_differentialDriveSim = new DifferentialDrivetrainSim(
        //             Constants.kDrivetrainPlant,
        //             Constants.kDriveGearbox,
        //             Constants.kDriveGearing,
        //             Constants.kTrackwidth,
        //             Constants.kWheelDiameterMeters / 2.0,
        //             null);

        //     m_leftLeader_sim = m_leftLeader.getSimCollection();
        //     m_rightLeader_sim = m_rightLeader.getSimCollection();
        //             // m_leftEncoderSim = new EncoderSim(m_leftEncoder);
        //     // m_rightEncoderSim = new EncoderSim(m_rightEncoder);
        //     m_gyroAngleSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");

        //     m_fieldSim = new Field2d();
        //     SmartDashboard.putData("Field", m_fieldSim);
        // }
    }

    public void setMotorMode(NeutralModeValue m) {
        m_leftMotor.setNeutralMode(m);
        m_rightMotor.setNeutralMode(m);
    }

    // Get the current set speed of the speed controllers
    public double getRightSpeed() {
        return -m_rightMotor.get();
    }

    public double getLeftSpeed() {
        return m_leftMotor.get();
    }

    // Get stats about the encoders
    public double getLeftEncoderDistance() {
        // return m_leftEncoder.getDistance();
        return m_leftMotor.getPosition().getValueAsDouble() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT;
    }
    public void setLeftEncoderDistance(double distance) {
        m_leftMotor.setPosition((int) (distance / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    }

    public double getRightEncoderDistance() {
        // return m_rightEncoder.getDistance();
        return -m_rightMotor.getPosition().getValueAsDouble() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT;
    }
    public void setRightEncoderDistance(double distance) {
        m_leftMotor.setPosition((int) (-distance / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    }
    public double getDistance() {
        return 0.5 * (getLeftEncoderDistance() + getRightEncoderDistance());
    }

    public double getLeftEncoderVelocity() {
        // sensor velocity is per 100ms, so an extra scale of 10
        return m_leftMotor.getVelocity().getValueAsDouble() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT * 10.0;
    }
    public double getRightEncoderVelocity() {
        // sensor velocity is per 100ms, so an extra scale of 10
        return -m_rightMotor.getVelocity().getValueAsDouble() * Constants.DRIVE_FALCON_DISTANCE_PER_UNIT * 10.0;
    }

    public int getLeftEncoderTicks() {
        // return m_leftEncoder.get();
        return (int)m_leftMotor.getPosition().getValueAsDouble();
    }
    public void setLeftEncoderTicks(int ticks){
        // m_leftLeader.setSelectedSensorPosition(ticks);
        m_leftMotor.setPosition(ticks);
    }

    public int getRightEncoderTicks() {
        // return m_rightEncoder.get();
        // return (int)m_rightLeader.getSelectedSensorPosition();
        return (int)m_rightMotor.getPosition().getValueAsDouble();
    }
    public void setRightEncoderTicks(int ticks){
        m_rightMotor.setPosition(ticks);
    }

    // // Get and Set odometry values
    // public Pose2d getPose() {
    //     return m_odometry.getPoseMeters();
    // }

    // public void setPose(Pose2d pose) {
    //     // m_leftEncoder.reset();
    //     // m_rightEncoder.reset();
    //     // m_leftLeader.setSelectedSensorPosition(0.0);
    //     // m_rightLeader.setSelectedSensorPosition(0.0);
    //     setLeftEncoderTicks(0);
    //     setRightEncoderTicks(0);

    //     if (Robot.isSimulation()) m_differentialDriveSim.setPose(new Pose2d()); // drive sim doesn't seem to get reset anymore?
    //     m_odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), getLeftEncoderDistance(), getRightEncoderDistance(), pose);
    // }

    // public double getHeading() {
    //     return m_odometry.getPoseMeters().getRotation().getDegrees();
    // }

    // // Get Gyro info
    // public double getGyroAngle() {
    //     return Math.IEEEremainder(m_navX.getAngle(), 360) * -1;
    // }

    @Override
    public void periodic() {
        // m_odometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftEncoderDistance(),
        //         getRightEncoderDistance());

        // SmartDashboard.putNumber("driveTrain/heading", getHeading());
        // SmartDashboard.putNumber("driveTrain/NavX gyro", getGyroAngle());
        // SmartDashboard.putNumber("driveTrain/x position", getPose().getX());
        // SmartDashboard.putNumber("driveTrain/y position", getPose().getY());

        // SmartDashboard.putNumber("driveTrain/left encoder", getLeftEncoderTicks());
        // SmartDashboard.putNumber("driveTrain/right encoder", getRightEncoderTicks());
        // SmartDashboard.putNumber("driveTrain/left distance", getLeftEncoderDistance());
        // SmartDashboard.putNumber("driveTrain/right distance", getRightEncoderDistance());

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
        m_leftMotor.setVoltage(leftVolts);
        m_rightMotor.setVoltage(rightVolts);
        m_differentialDrive.feed();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        // return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    // @Override
    // public void simulationPeriodic() {
    //     // Set the inputs to the system. Note that we need to convert
    //     // the [-1, 1] PWM signal to voltage by multiplying it by the
    //     // robot controller voltage.
    //     m_differentialDriveSim.setInputs(m_leftMotors.get() * RobotController.getInputVoltage(),
    //             m_rightMotors.get() * RobotController.getInputVoltage());

    //     // Advance the model by 20 ms. Note that if you are running this
    //     // subsystem in a separate thread or have changed the nominal timestep
    //     // of TimedRobot, this value needs to match it.
    //     m_differentialDriveSim.update(0.02);

    //     // Update all of our sensors.
    //     /*
    //     m_leftEncoderSim.setDistance(m_differentialDriveSim.getLeftPositionMeters());
    //     m_leftEncoderSim.setRate(m_differentialDriveSim.getLeftVelocityMetersPerSecond());
    //     m_rightEncoderSim.setDistance(m_differentialDriveSim.getRightPositionMeters());
    //     m_rightEncoderSim.setRate(m_differentialDriveSim.getRightVelocityMetersPerSecond());
    //     */

    //     m_leftLeader_sim.setIntegratedSensorRawPosition((int) (m_differentialDriveSim.getLeftPositionMeters() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    //     m_leftLeader_sim.setIntegratedSensorVelocity((int) (m_differentialDriveSim.getLeftVelocityMetersPerSecond() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT / 10.0));

    //     m_rightLeader_sim.setIntegratedSensorRawPosition( - (int) (m_differentialDriveSim.getRightPositionMeters() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT));
    //     m_rightLeader_sim.setIntegratedSensorVelocity( - (int) (m_differentialDriveSim.getRightVelocityMetersPerSecond() / Constants.DRIVE_FALCON_DISTANCE_PER_UNIT / 10.0));

    //     m_gyroAngleSim.set(-m_differentialDriveSim.getHeading().getDegrees());
    //     m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    // }

    // public Field2d getField2d() {
    //     return m_fieldSim;
    // }

    // public double disToTurn(double angle){
    //     // calculate the distance one side of the wheels need to turn to get to the desired angle
    //     return Constants.kTrackwidth*Math.PI*(angle / (2*Math.PI));
    // }

    // public void setSetPoint(boolean turnToLeft, TrapezoidProfile.State setPoint, double startDisLeft, double startDisRight) {
    //     // if turn to left, let the right motor drive, and vice versa
    //     if(turnToLeft){
    //         // have the left side go forward and right side backward to turn the robot to the right
    //         m_leftLeader.set(ControlMode.Position, startDisLeft + setPoint.position);
    //         m_rightLeader.set(ControlMode.Position, startDisRight - setPoint.position);

    //         SmartDashboard.putNumber("DriveTrain/setPointLeft", Units.metersToInches(startDisLeft + setPoint.position));
    //         SmartDashboard.putNumber("DriveTrain/setPointRight", Units.metersToInches(startDisRight - setPoint.position));
    //     }else{
    //         // have the right side go forward and left side backward to turn the robot to the left
    //         m_leftLeader.set(ControlMode.Position, startDisLeft - setPoint.position);
    //         m_rightLeader.set(ControlMode.Position, startDisRight + setPoint.position);

    //         SmartDashboard.putNumber("DriveTrain/setPointLeft", Units.metersToInches(startDisLeft - setPoint.position));
    //         SmartDashboard.putNumber("DriveTrain/setPointRight", Units.metersToInches(startDisRight + setPoint.position));
    //     }
    //     SmartDashboard.putBoolean("DriveTrain/turnToLeft", turnToLeft);        
    // }

    // public double turnSpeedCalc(double angleError) {
    //     double absErr = Math.abs(angleError);
    //     double turnSpeed;
    //     if (absErr > 60.0) {
    //         turnSpeed = 0.6;//0.8;
    //     }
    //     else if (absErr > 30.0) {
    //         turnSpeed = 0.15;//0.2; //0.4;
    //     }
    //     else if (absErr > 10.0) {
    //         turnSpeed = 0.1;//0.15;
    //     }
    //     // else if (absErr > 5.0) {
    //     //     turnSpeed = 0.1; //0.07;
    //     // }
    //     else {
    //         turnSpeed = 0.125; // 0.065;
    //     }

    //     return turnSpeed * Math.signum(angleError);
    // }
}
