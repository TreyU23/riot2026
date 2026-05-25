package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final SparkMax m_leftMotor = new SparkMax(ShooterConstants.kLeftMotorID, MotorType.kBrushless);
    private final SparkMax m_rightMotor = new SparkMax(ShooterConstants.kRightMotorID, MotorType.kBrushless);

    private final RelativeEncoder m_leftEnc = m_leftMotor.getEncoder();
    private final RelativeEncoder m_rightEnc = m_rightMotor.getEncoder();

    private final SparkClosedLoopController m_leftPID = m_leftMotor.getClosedLoopController();
    private final SparkClosedLoopController m_rightPID = m_rightMotor.getClosedLoopController();

    private static double m_spinSetpoint = 0.0;
    private static double m_velocitySetpoint = 0.0;
    private static double m_adjust = 0.0;

    public Shooter() {
        motorConfigs();
    }

    public double getLeftVelocity() {
        return m_leftEnc.getVelocity();
    }

    public double getRightVelocity() {
        return m_rightEnc.getVelocity();
    }

    public double getLeftCurrent() {
        return m_leftMotor.getOutputCurrent();
    }

    public double getRightCurrent() {
        return m_rightMotor.getOutputCurrent();
    }

    public void set(double spin, double velocity) {
        double spinReq = 0.01 * spin * velocity;

        m_velocitySetpoint = velocity;
        m_spinSetpoint = spinReq;

        m_velocitySetpoint += m_adjust;

        if (m_velocitySetpoint >= 100.0) m_velocitySetpoint = 100.0;
        if (m_velocitySetpoint <= -10.0) m_velocitySetpoint = -10.0;

        double leftSetpoint = m_velocitySetpoint + m_spinSetpoint / 2.0;
        double rightSetpoint = -1.0 * (m_velocitySetpoint - m_spinSetpoint / 2.0);

        m_leftPID.setSetpoint(leftSetpoint, ControlType.kVelocity);
        m_rightPID.setSetpoint(rightSetpoint, ControlType.kVelocity);
    }

    public void stop() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }
    
    public Command setCmd(double spin, double velocity) {
        return runEnd(()-> set(spin, velocity), ()-> stop());
    }

    public void adjust(double adj) {
        m_adjust += adj;
    }

    public Command adjustCmd(double adj) {
        return runOnce(()-> adjust(adj));
    }

    public void motorConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.closedLoop.pid(ShooterConstants.kP, 
                                ShooterConstants.kI, 
                                    ShooterConstants.kD);

        m_cfg.closedLoop.feedForward.kS(ShooterConstants.kS);
        m_cfg.closedLoop.feedForward.kV(ShooterConstants.kV);

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kShooter);
        m_cfg.idleMode(IdleMode.kBrake);

        m_leftMotor.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_rightMotor.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Veloicity", getLeftVelocity());
        SmartDashboard.putNumber("Shooter Setpoint", m_velocitySetpoint);
        SmartDashboard.putNumber("Shooter Current", getLeftCurrent());
        SmartDashboard.putNumber("Shooter Adjust", m_adjust);
    }
}