package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldInformation;

public class Vision extends SubsystemBase {
    public enum VisionMode {
        INTAKE,
        SHOOTER,
        HUBFINDER,
    }

    public static final VisionMode DEFAULT_MODE = VisionMode.INTAKE;
    private static final double[] EMPTY_TARGET_INFO = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    private Relay m_relay;
    private DriveTrain m_driveTrain;
    private double[] m_targetInfoSim = EMPTY_TARGET_INFO.clone();
    private double[] m_lastResult = EMPTY_TARGET_INFO;
    private double[] m_lastSuccessfulResult = EMPTY_TARGET_INFO;

    private static final double HORIZONTAL_HALF_FOV = Math.toRadians(32.0);

    // Note: driveTrain is needed for simulation mode only

    public Vision(DriveTrain driveTrain) {
        m_relay = new Relay(0);
        m_driveTrain = driveTrain;

        // start the camera in hubfinder for auto, but don't turn on the LED
        this.setMode(VisionMode.HUBFINDER, false);
    }

    @Override
    public void periodic() {
        m_lastResult = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        if (m_lastResult[1] > 0.5) {
            m_lastSuccessfulResult = m_lastResult;
        }
    }

    @Override
    public void simulationPeriodic() {
        if (getMode() == VisionMode.HUBFINDER) {
            Pose2d robotpos = m_driveTrain.getPose();
            double distance = robotpos.getTranslation().minus(FieldInformation.fieldCenter).getNorm();
            Translation2d robotToHub = FieldInformation.fieldCenter.minus(robotpos.getTranslation());
            Rotation2d angle = new Rotation2d(robotToHub.getX(), robotToHub.getY()).minus(robotpos.getRotation());
            double angleRadians = angle.getRadians();
            if(Math.abs(angleRadians) < HORIZONTAL_HALF_FOV) {
                m_targetInfoSim[1] = 1.0;
                m_targetInfoSim[3] = Units.metersToInches(distance);
                m_targetInfoSim[4] = -angleRadians;
            } else {
                m_targetInfoSim[1] = 0.0;
            }
        } else {
            m_targetInfoSim[1] = 0.0;
        }
        m_targetInfoSim[0] = Timer.getFPGATimestamp();

        SmartDashboard.putNumberArray("vision/target_info", m_targetInfoSim);
    }

    // set vision processing mode using enum, and set LED to match what is needed
    public void setMode(VisionMode mode) {
        setMode(mode, mode == VisionMode.HUBFINDER);
    }

    // set vision processing mode, and full control of LED
    // should be used from outside only for special cases
    public void setMode(VisionMode mode, boolean led) {
        SmartDashboard.putString("vision/active_mode/selected", mode.name().toLowerCase());
        setLedRing(led);
    }
    
    // what mode is the vision processing running in
    public VisionMode getMode() {
        String mode = SmartDashboard.getString("vision/active_mode/selected", "");
        try {
            return VisionMode.valueOf(mode.toUpperCase());
        } catch(Exception e) {
            // ignore
        }
        return DEFAULT_MODE;
    }

    public boolean getStatus() {
        // 1 = success, but it comes as a float, so test with a greater-than
        return m_lastResult[1] > 0.5;
    }
    
    public double getDistance() {
        return m_lastSuccessfulResult[3];
    }
    
    public double getRobotAngle() {
        return Math.toDegrees(m_lastSuccessfulResult[4]);
    }

    public void setLedRing (boolean on) {
        m_relay.set(on ? Value.kReverse : Value.kOff);
    }
}
