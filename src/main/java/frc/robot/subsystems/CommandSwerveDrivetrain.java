package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.constants.DriveConstants;
import frc.robot.utilities.ChassisAcceleration;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class CommandSwerveDrivetrain extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFLAzimuth, DriveConstants.kFLDrive);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFRAzimuth, DriveConstants.kFRDrive);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBLAzimuth, DriveConstants.kBLDrive);
  private final SwerveModule backRigt = new SwerveModule(DriveConstants.kBRAzimuth, DriveConstants.kBRDrive);

  private static AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private static SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;

  private SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kSlewX);
  private SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kSlewY);
  private SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kSlewRot);
  private final double[] m_slew = {0.0, 0.0, 0.0};

  private ChassisSpeeds m_lastDriveSpeed = new ChassisSpeeds();
  private ChassisAcceleration m_driveAccel = new ChassisAcceleration();

  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
      m_gyro.getRotation2d(), getModulePositions());

  private SwerveModuleState[] m_setStates = m_kinematics
      .toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));

  public CommandSwerveDrivetrain() {
    resetOdometry(getGyro());
    m_gyro.reset();
  }

  public void fieldRelative(double xSpeed, double ySpeed, double rot) {
    double tolerance = 0.05;

    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);
    rot = m_slewRot.calculate(rot);

    m_slew[0] = xSpeed;
    m_slew[1] = ySpeed;
    m_slew[2] = rot;

    if (Math.abs(rot) < tolerance) rot = 0.0;
    if (Math.abs(xSpeed) < tolerance) xSpeed = 0.0;
    if (Math.abs(ySpeed) < tolerance) ySpeed = 0.0;

    setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()));
  }

  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), getModulePositions());
  }

  public void resetOdometry(Rotation2d angle) {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(angle.getDegrees());
    
    m_odometry.resetPosition(m_gyro.getRotation2d().times(-1.0), 
      getModulePositions(), new Pose2d(getPose().getTranslation(), angle));
  }

  public void changeSlewRate(double translation, double rotation) {
    m_slewX = new SlewRateLimiter(translation, -translation, m_slew[0]);
    m_slewY = new SlewRateLimiter(translation, -translation, m_slew[1]);
    m_slewRot = new SlewRateLimiter(rotation, -rotation, m_slew[2]);
  }

  @Override
  public void periodic() {
    ChassisSpeeds m_speeds = getChassisSpeed();

    m_driveAccel = new ChassisAcceleration(m_speeds, m_lastDriveSpeed, 0.2);
    m_lastDriveSpeed = m_speeds;

    m_fieldRelVel = new FieldRelativeSpeed(getChassisSpeed(), getGyro());
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, 0.2);
    m_lastFieldRelVel = m_fieldRelVel;

    double xSpeed = getChassisSpeed().vxMetersPerSecond;
    double ySpeed = getChassisSpeed().vyMetersPerSecond;

    double speed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

    SmartDashboard.putNumber("Speed", speed);

    updateOdometry();
    getPose();

    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

    Translation2d rotAdj = translation.rotateBy(new Rotation2d(-Math.PI / 2.0))
        .times(chassisSpeeds.omegaRadiansPerSecond * DriveConstants.kRotationTranslation);

    translation = translation.plus(rotAdj);

    ChassisSpeeds adjust = new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);

    SwerveModuleState[] desiredStates = m_kinematics.toSwerveModuleStates(adjust);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeed);

    m_setStates = desiredStates;

    frontLeft.setModule(m_setStates[0]);
    frontRight.setModule(m_setStates[1]);
    backLeft.setModule(m_setStates[2]);
    backRigt.setModule(m_setStates[3]);
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRigt.stop();
  }

  public ChassisAcceleration getChassisAccel() {
    return m_driveAccel;
  }

  public FieldRelativeSpeed getFieldRelativeSpeed() {
    return m_fieldRelVel;
  }

  public FieldRelativeAccel getFieldRelativeAccel() {
    return m_fieldRelAccel;
  }

  public double getSpeed() {
    double xSpeed = getChassisSpeed().vxMetersPerSecond;
    double ySpeed = getChassisSpeed().vyMetersPerSecond;

    double speed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

    return speed;
  }

  public Rotation2d getGyro() {
    return m_gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeed() {
    return m_kinematics.toChassisSpeeds(
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRigt.getState());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {frontLeft.getModulePosition(), frontRight.getModulePosition(), 
                                        backLeft.getModulePosition(), backRigt.getModulePosition()};
  }
}