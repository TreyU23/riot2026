package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class DriveConstants {
    public static final int kGyro = 0;

    public static final int kFLDrive = 1;
    public static final int kFLAzimuth = 2;
    public static final Translation2d kFLPosition = new Translation2d();

    public static final int kFRDrive = 3;
    public static final int kFRAzimuth = 4;
    public static final Translation2d kFRPosition = new Translation2d();

    public static final int kBLDrive = 5;
    public static final int kBLAzimuth = 6;
    public static final Translation2d kBLPosition = new Translation2d();

    public static final int kBRDrive = 7;
    public static final int kBRAzimuth = 8;
    public static final Translation2d kBRPosition = new Translation2d();

    public static final double kSlewX = 0.0;
    public static final double kSlewY = 0.0;
    public static final double kSlewRot = 0.0;

    public static final double kMaxSpeed = 10.0;
    public static final double kMaxAcceleration = 15.0;
    public static final double kRotationTranslation = 0.0;

    public static final SwerveDriveKinematics kDriveKinematics = 
            new SwerveDriveKinematics(kFLPosition, kFRPosition, kBLPosition, kBRPosition);

    public class azimuth {
        public static final double kVoltageCompensation = 0.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;

        public static final double kGearRatio = 25.0;
    }

    public class drive {
        public static final double kVoltageCompensation = 0.0;

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
    }
}
