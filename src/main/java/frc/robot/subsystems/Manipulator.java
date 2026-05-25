package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CurrentLimitConstants;
import frc.robot.constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
    private final SparkMax m_manipulator = new SparkMax(ManipulatorConstants.kMotorID, MotorType.kBrushless);

    private final SparkClosedLoopController m_pid = m_manipulator.getClosedLoopController();

    private final RelativeEncoder m_encoder = m_manipulator.getEncoder();

    private static double m_setpoint = 0.0;

    public Manipulator() {
        motorConfigs();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public double getCurrent() {
        return m_manipulator.getOutputCurrent();
    }

    public void set(double pose) {
        m_setpoint = pose;
        m_pid.setSetpoint(m_setpoint, ControlType.kPosition);
    }

    public void stop() {
        m_manipulator.stopMotor();
    }

    public Command setCmd(double pose) {
        return runEnd(()-> set(pose), ()-> stop());
    }

    public void motorConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.closedLoop.pid(ManipulatorConstants.kP, 
                                ManipulatorConstants.kI, 
                                    ManipulatorConstants.kD);

        m_cfg.idleMode(IdleMode.kBrake);

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kManipulator);

        m_cfg.voltageCompensation(ManipulatorConstants.kVoltageCompensation);

        m_manipulator.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pithcer Position", getPosition());
        SmartDashboard.putNumber("Pitcher Setpoint", m_setpoint);
        SmartDashboard.putNumber("Pitcher Current", getCurrent());
    }
}