package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
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
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final SparkMax m_intake = new SparkMax(IntakeConstants.Intake.kMotorID, MotorType.kBrushless);
    private final SparkMax m_kicker = new SparkMax(IntakeConstants.Kicker.kMotorID, MotorType.kBrushless);

    private final RelativeEncoder m_intakeEnc = m_intake.getEncoder();
    private final RelativeEncoder m_kickerEnc = m_kicker.getEncoder();

    private final SparkClosedLoopController m_intakePID = m_intake.getClosedLoopController();
    private final SparkClosedLoopController m_kickerPID = m_kicker.getClosedLoopController();
    
    private static double m_setpoint = 0.0;
    
    public Intake() {
        intakeConfigs();
        kickerConfigs();
    }

    public double getIntakeCurrent() {
        return m_intake.getOutputCurrent();
    }

    public double getKickerCurrent() {
        return m_kicker.getOutputCurrent();
    }

    public double getIntakeVelocity() {
        return m_intakeEnc.getVelocity();
    }

    public double getKickerVelocity() {
        return m_kickerEnc.getVelocity();
    }

    public void set(double req) {
        m_setpoint = req;
        m_intakePID.setSetpoint(m_setpoint, ControlType.kVelocity);
        m_kickerPID.setSetpoint(m_setpoint*0.7, ControlType.kVelocity);
    }

    public void stop() {
        m_intake.stopMotor();
        m_kicker.stopMotor();
    }

    public Command setCmd(double req, double robotVelocity) {
        return runEnd(()-> {
            if (robotVelocity <= 0.0) set(req);
            else set(req*-1.0); 
        }, ()-> stop());
    }

    public void intakeConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.closedLoop.pid(IntakeConstants.Intake.kP, 
                                IntakeConstants.Intake.kI, 
                                    IntakeConstants.Intake.kD);

        m_cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        m_cfg.idleMode(IdleMode.kBrake);

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kIntake);

        m_intake.configure(m_cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void kickerConfigs() {
        SparkMaxConfig m_cfg = new SparkMaxConfig();

        m_cfg.smartCurrentLimit(CurrentLimitConstants.kKicker);

        m_cfg.idleMode(IdleMode.kBrake);

        m_cfg.voltageCompensation(IntakeConstants.Kicker.kVoltageCompensation);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", getIntakeCurrent());
        SmartDashboard.putNumber("Kicker Current", getKickerCurrent());
        SmartDashboard.putNumber("Intake Velocity", getIntakeVelocity());
        SmartDashboard.putNumber("Kicker Velocity", getKickerVelocity());
        SmartDashboard.putNumber("Intake Setpoint", m_setpoint);
        SmartDashboard.putNumber("Kicker Setpoint", m_setpoint*0.7);
    }
}