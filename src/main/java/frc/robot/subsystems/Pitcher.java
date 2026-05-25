package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.PitcherConstants;

public class Pitcher extends SubsystemBase {
    private final SparkMax m_pitcher = new SparkMax(PitcherConstants.kMotorID, MotorType.kBrushless);

    private final SparkClosedLoopController m_pid = m_pitcher.getClosedLoopController();

    private final RelativeEncoder m_encoder = m_pitcher.getEncoder();

    private static double m_setpoint = 0.0;
    private static double m_adjust = 0.0;

    public Pitcher() {
        motorConfigs();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getCurrent() {
        return m_pitcher.getOutputCurrent();
    }

    public void set(double pose) {
        m_setpoint = pose;
        m_setpoint += m_adjust;
        m_pid.setSetpoint(m_setpoint, ControlType.kPosition);
    }

    public void stop() {
        m_pitcher.stopMotor();
    }

    public Command pitchCmd(double pose) {
        return runEnd(()-> set(pose), ()-> stop());
    }

    public void adjust(double adj) {
        m_adjust = adj;
    }

    public Command adjustCmd(double adj) {
        return runOnce(()-> adjust(adj));
    }

    public Command zeroCmd() {
        return runEnd(()-> {
            m_pitcher.set(-1.0);
        }, ()-> {
            stop();
            m_encoder.setPosition(0.0);
        });
    }

    public void motorConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.closedLoop.pid(PitcherConstants.kP, 
                                PitcherConstants.kI, 
                                    PitcherConstants.kD);

        m_cfg.softLimit.forwardSoftLimit(PitcherConstants.kMaxPosition)
                        .forwardSoftLimitEnabled(true)
                        .reverseSoftLimit(PitcherConstants.kMinPosition)
                        .reverseSoftLimitEnabled(true);

        m_cfg.idleMode(IdleMode.kBrake);

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kPithcer);

        m_cfg.voltageCompensation(PitcherConstants.kVoltageCompensation);

        m_pitcher.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pithcer Position", getPosition());
        SmartDashboard.putNumber("Pitcher Setpoint", m_setpoint);
        SmartDashboard.putNumber("Pitcher Current", getCurrent());
        SmartDashboard.putNumber("Pithcer Adjust", m_adjust);
    }
}