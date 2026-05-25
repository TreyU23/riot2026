package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase {
    private final SparkMax m_indexer = new SparkMax(IndexerConstants.kMotorID, MotorType.kBrushless);
    
    private static double m_setpoint = 0.0;

    public Indexer() {
        motorConfigs();
    }

    public double getDutyCyle() {
        return m_indexer.getAppliedOutput();
    }

    public double getCurrent() {
        return m_indexer.getOutputCurrent();
    }

    public void set(double percent) {
        m_setpoint = percent;
        m_indexer.set(m_setpoint);
    }

    public void stop() {
        m_indexer.stopMotor();
    }

    public Command setCmd(double percent) {
        return runEnd(()-> set(percent), ()-> stop());
    }

    public void motorConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.voltageCompensation(IndexerConstants.kVoltageCompensation);

        m_cfg.idleMode(IdleMode.kBrake);

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kIndexer);

        m_indexer.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer DutyCyle", getDutyCyle());
        SmartDashboard.putNumber("Indexer Setpoint", m_setpoint);
        SmartDashboard.putNumber("Indexer Current", getCurrent());
    }
}