package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldInformation {
    public static final Translation2d fieldCenter = new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162));
    static Translation2d centerAngle(double distanceMeters, Rotation2d angle){
        return new Translation2d(distanceMeters, angle).plus(fieldCenter);
    }
    static Pose2d centerAnglePose(double distanceMeters, Rotation2d angle){
        return new Pose2d(centerAngle(distanceMeters, angle), angle.plus(Rotation2d.fromDegrees(180)));
    }
    public static final Pose2d upperBlueStart = centerAnglePose(Units.inchesToMeters(90), Rotation2d.fromDegrees(148));
    public static final Pose2d middleBlueStart = centerAnglePose(Units.inchesToMeters(90), Rotation2d.fromDegrees(225));
    public static final Pose2d lowerBlueStart = centerAnglePose(Units.inchesToMeters(90), Rotation2d.fromDegrees(261));
    public static final Pose2d middleBlueBall = centerAnglePose(Units.inchesToMeters(157), Rotation2d.fromDegrees(215));
    public static final Pose2d cornerBlueBall = centerAnglePose(Units.inchesToMeters(291), Rotation2d.fromDegrees(202));
    public static final Pose2d lowerBlueBall = centerAnglePose(Units.inchesToMeters(127), Rotation2d.fromDegrees(263));
}