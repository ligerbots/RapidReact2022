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
    public static Pose2d centerAnglePose(double distanceMeters, Rotation2d angle){
        return new Pose2d(centerAngle(distanceMeters, angle), angle.plus(Rotation2d.fromDegrees(180)));
    }
    public static Translation2d ballTranslation(Pose2d pose, double x, double y){
        Translation2d translation = pose.getTranslation();
        return new Translation2d(translation.getX() + Units.inchesToMeters(x), translation.getY() + Units.inchesToMeters(y));
    }
    public static Pose2d ballPose(Pose2d pose, double x, double y){
        return new Pose2d(
            ballTranslation(pose, x, y),
            pose.getRotation()
        );
    }
    public static Translation2d ballTranslationPolar(Pose2d pose, double radius, double angle){
        double radian = Units.degreesToRadians(angle);
        return ballTranslation(pose, radius * Math.cos(radian), radius*Math.sin(radian));
    }
    public static Pose2d ballPosePolar(Pose2d pose, double radius, double angle){
        double radian = Units.degreesToRadians(angle);
        return ballPose(pose, radius * Math.cos(radian), radius*Math.sin(radian));
    }
    public static final Pose2d upperBlueStart = centerAnglePose(Units.inchesToMeters(90), Rotation2d.fromDegrees(148));
    public static final Pose2d middleBlueStart = centerAnglePose(Units.inchesToMeters(90), Rotation2d.fromDegrees(225));
    public static final Pose2d lowerBlueStart = centerAnglePose(Units.inchesToMeters(90), Rotation2d.fromDegrees(261));
    public static final Pose2d middleBlueBall = centerAnglePose(Units.inchesToMeters(150), Rotation2d.fromDegrees(215));
    public static final Pose2d cornerBlueBall = centerAnglePose(Units.inchesToMeters(300), Rotation2d.fromDegrees(202.5));
    public static final Pose2d lowerBlueBall = centerAnglePose(Units.inchesToMeters(150), Rotation2d.fromDegrees(260));
}