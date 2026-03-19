// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.thethriftybot.devices.ThriftyNova;
import com.thethriftybot.devices.ThriftyNova.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants.ElevatorConstants;
import frc.robot.Constants.ClimbConstants.TongueConstants;
import frc.robot.Constants.MechanismPositionConstants;
import frc.robot.Robot;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class TongueSubsystem extends SubsystemBase {

    private final ThriftyNova m_motor;
    private final SmartMotorController m_smartMotorController;
    private final Elevator m_tongue;

    private SmartMotorControllerConfig motorConfig;

    public TongueSubsystem() {
        m_motor = new ThriftyNova(TongueConstants.MOTOR_CAN_ID, MotorType.NEO);

        motorConfig = new SmartMotorControllerConfig(this)
                .withMechanismCircumference(
                        ElevatorConstants.CHAIN_PITCH.times(ElevatorConstants.TOOTH_COUNT))
                .withClosedLoopController(
                        TongueConstants.PID_kP,
                        TongueConstants.PID_kI,
                        TongueConstants.PID_kD,
                        TongueConstants.MAX_VELOCITY,
                        TongueConstants.MAX_ACCELERATION)
                .withSoftLimit(TongueConstants.SOFT_LIMIT_MIN, TongueConstants.SOFT_LIMIT_MAX)
                .withGearing(new MechanismGearing(TongueConstants.GEARBOX))
                .withIdleMode(MotorMode.COAST)
                .withTelemetry("TongueMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(TongueConstants.STATOR_CURRENT_LIMIT)
                .withMotorInverted(false)
                .withClosedLoopRampRate(TongueConstants.CLOSED_LOOP_RAMP_RATE)
                .withFeedforward(TongueConstants.FEEDFORWARD)
                .withControlMode(ControlMode.CLOSED_LOOP);

        m_smartMotorController = new NovaWrapper(m_motor, DCMotor.getNeo550(1), motorConfig);

        MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
                .withMaxRobotHeight(MechanismPositionConstants.ROBOT_MAX_HEIGHT)
                .withMaxRobotLength(MechanismPositionConstants.ROBOT_MAX_LENGTH)
                .withRelativePosition(TongueConstants.RELATIVE_POSITION);

        ElevatorConfig tongueConfig = new ElevatorConfig(m_smartMotorController)
                .withHardLimits(TongueConstants.HARD_LIMIT_MIN, TongueConstants.HARD_LIMIT_MAX)
                .withTelemetry("Tongue", TelemetryVerbosity.HIGH)
                .withMechanismPositionConfig(robotToMechanism)
                .withMass(TongueConstants.MECHANISM_MASS);

        if (Robot.isSimulation()) {
            tongueConfig.withStartingHeight(TongueConstants.STARTING_HEIGHT);
        }

        m_tongue = new Elevator(tongueConfig);
    }

    /**
     * Creates a SysId characterization command for the tongue.
     *
     * @return the SysId command
     */
    public Command sysId() {
        return m_tongue.sysId(
                Volts.of(12), Volts.of(1).per(Second), Second.of(15))
                .beforeStarting(
                        () -> SignalLogger.start())
                .finallyDo(() -> SignalLogger.stop());
    }

    public Command elevCmd(double dutycycle) {
        return m_tongue.set(dutycycle);
    }

    public Command setDistance(Distance distance) {
        return m_tongue.setHeight(distance);
    }

    public Command stop() {
        return setDistance(m_tongue.getHeight());
    }

    public Optional<Distance> getSetpoint() {
        Optional<Angle> angleSetpoint = m_tongue.getMechanismSetpoint();
        if (!angleSetpoint.isPresent()) {
            return Optional.empty();
        }
        return Optional.of(motorConfig.convertFromMechanism(angleSetpoint.get()));
    }

    @Override
    public void periodic() {
        m_tongue.updateTelemetry();

        if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber("Tongue/position (Inch)", m_tongue.getHeight().in(Inches));
            SmartDashboard.putNumber("Tongue/setpoint (Inch)",
                    getSetpoint().map(pos -> pos.in(Inches)).orElse(Double.NaN));

        }
    }

    @Override
    public void simulationPeriodic() {
        m_tongue.simIterate();
    }
}
