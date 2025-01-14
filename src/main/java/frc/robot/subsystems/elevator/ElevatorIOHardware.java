package frc.robot.subsystems.elevator;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Value;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.PhoenixUtil.*;

public class ElevatorIOHardware implements ElevatorIO
{
    private SparkMax        _leaderMotor;
    private SparkMax        _followerMotor;
    private RelativeEncoder _extensionEncoder;

    public ElevatorIOHardware()
    {
        _leaderMotor      = new SparkMax(Constants.CAN.LEAD_ELEVATOR, MotorType.kBrushless);
        _followerMotor    = new SparkMax(Constants.CAN.FOLLOWER_ELEVATOR, MotorType.kBrushless);
        _extensionEncoder = _leaderMotor.getEncoder();

        var leaderConfig = new SparkMaxConfig();

        leaderConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        var followerConfig = new SparkMaxConfig();

        followerConfig.idleMode(IdleMode.kBrake).follow(Constants.CAN.LEAD_ELEVATOR).inverted(true).voltageCompensation(12);
        followerConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        tryUntilOk(_leaderMotor, 5, () -> _leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(_followerMotor, 5, () -> _followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        ifOk(_leaderMotor, _extensionEncoder::getPosition, (value) -> inputs.extensionPosition = value);
        ifOk(_leaderMotor, _extensionEncoder::getVelocity, (value) -> inputs.extensionVelocity = value);
        ifOk(_leaderMotor, new DoubleSupplier[] { _leaderMotor::getAppliedOutput, _leaderMotor::getBusVoltage }, (values) -> inputs.leaderVolts = values[0] * values[1]);
    }
}
