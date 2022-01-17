package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public enum VisionMode {
        INTAKE,
        SHOOTER,
        HUBFINDER,
    }

    private static final double[] EMPTY_TARGET_INFO = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0};


    private Relay m_spike;
    private DriveTrain m_driveTrain;



    // Note: driveTrain is needed for simulation mode only

    public Vision(DriveTrain driveTrain) {
        m_spike = new Relay(0);
        m_driveTrain = driveTrain;

        // start the camera in hubfinder for auto, but don't turn on the LED
        this.setMode(VisionMode.HUBFINDER, false);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        if (getMode() == VisionMode.HUBFINDER) {
            // TODO: implement simulation
        }
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
        return VisionMode.INTAKE;
    }

    public boolean getStatus() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        // 1 = success, but it comes as a float, so test with a greater-than
        return visionData[1] > 0.5;
    }
    
    public double getDistance() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        return visionData[3];
    }
    
    public double getRobotAngle() {
        double[] visionData = SmartDashboard.getNumberArray("vision/target_info", EMPTY_TARGET_INFO);
        return Math.toDegrees(visionData[4]);
    }

    public void setLedRing (boolean on) {
        m_spike.set(on ? Value.kReverse : Value.kOff);
    }
}
