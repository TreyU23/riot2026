package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    private final SparkMax m_azmuith;
    private final SparkMax m_drive;

    private final SparkClosedLoopController m_azPID;
    private final SparkClosedLoopController m_drPID;

    public static double m_setPosition = 0.0;
    public static double m_setVelocity = 0.0;
    
    public SwerveModule(int azmuith, int drive) {
        m_azmuith = new SparkMax(azmuith, MotorType.kBrushless);
        m_drive = new SparkMax(drive, MotorType.kBrushless);

        m_azPID = m_azmuith.getClosedLoopController();
        m_drPID = m_drive.getClosedLoopController();

        azmuithConfigs();
        driveConfigs();
    }

    public double getVelocity() {
        return m_drive.getEncoder().getVelocity();
    }

    public Rotation2d getPosition() {
        return new Rotation2d(m_azmuith.getAbsoluteEncoder().getPosition());
    }

    public double getAzimuthCurrent() {
        return m_azmuith.getOutputCurrent();
    }

    public double getDriveCurrent() {
        return m_drive.getOutputCurrent();
    }

    public void setPosition(double pose) {
        m_azPID.setSetpoint(pose, ControlType.kPosition);
    }

    public void setVelocity(double vel) {
        m_drPID.setSetpoint(vel, ControlType.kVelocity);
    }

    public void stop() {
        m_drive.stopMotor();
        m_azmuith.stopMotor();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getPosition());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(m_drive.getEncoder().getPosition(), getPosition());
    }

    public void setModule(SwerveModuleState req) {
        req.optimize(getPosition());

        setPosition(req.angle.getRadians());
        setVelocity(req.speedMetersPerSecond);
    }

    public void azmuithConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.closedLoop.pid(DriveConstants.azimuth.kP, DriveConstants.azimuth.kI, DriveConstants.azimuth.kD);
        m_cfg.closedLoop.feedForward.kS(DriveConstants.azimuth.kS);
        m_cfg.closedLoop.feedForward.kV(DriveConstants.azimuth.kV);

        m_cfg.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        m_cfg.absoluteEncoder.positionConversionFactor(DriveConstants.azimuth.kGearRatio);
        m_cfg.absoluteEncoder.velocityConversionFactor(DriveConstants.azimuth.kGearRatio / 1000);

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kAzimuth);
        m_cfg.idleMode(IdleMode.kBrake);
        m_cfg.voltageCompensation(DriveConstants.azimuth.kVoltageCompensation);

        m_azmuith.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void driveConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.closedLoop.pid(DriveConstants.drive.kP, DriveConstants.drive.kI, DriveConstants.drive.kD);
        m_cfg.closedLoop.feedForward.kS(DriveConstants.drive.kS);
        m_cfg.closedLoop.feedForward.kV(DriveConstants.drive.kV);

        m_cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kDrive);
        m_cfg.idleMode(IdleMode.kBrake);
        m_cfg.voltageCompensation(DriveConstants.drive.kVoltageCompensation);

        m_azmuith.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("setPosition", m_setPosition);
        SmartDashboard.putNumber("setVelocity", m_setVelocity);
        SmartDashboard.putNumber("Position", getPosition().getRadians());
        SmartDashboard.putNumber("Velocity", getVelocity());
    }
}
