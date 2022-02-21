package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldInformation {
    public static Translation2d fieldCenter = new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162));
    static Translation2d centerAngle(double distanceMeters, Rotation2d angle){
        return new Translation2d(distanceMeters, angle).plus(fieldCenter);
    }
    static Pose2d centerAnglePose(double distanceMeters, Rotation2d angle){
        return new Pose2d(centerAngle(distanceMeters, angle), angle.plus(Rotation2d.fromDegrees(180)));
    }
    static Pose2d position1 = centerAnglePose(Units.inchesToMeters(80), Rotation2d.fromDegrees(35.2710435));
}

/**
 * 
 */