package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

// WPI timer doesnt do what we want, so this you quickly test for a delay
public class LigerTimer {
    double m_delayTime;
    double m_startTime;
    public LigerTimer(double delayTime) {
        m_delayTime = delayTime;
    }
    
    public static double time() {
        return Timer.getFPGATimestamp();
    }

    public void start() {
        m_startTime = time();
    }

    public boolean hasElapsed() {
        return (time() - m_startTime) >= m_delayTime;
    }
}
