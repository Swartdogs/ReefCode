package frc.robot.subsystems.drive;

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
import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.PhoenixUtil.*;

public class ModuleIOHardware implements ModuleIO
{
    private final Rotation2d _zeroRotation;

    // Hardware objects
    private final TalonFX         _driveTalon;
    private final SparkBase       _turnSpark;
    private final AbsoluteEncoder _turnEncoder;
    // TODO: Add in absolute encoder

    // Voltage control requests (Talon)
    private final VoltageOut      _voltageRequest         = new VoltageOut(0);
    private final VelocityVoltage _velocityVoltageRequest = new VelocityVoltage(0.0);

    // Closed loop controllers (Spark)
    private final SparkClosedLoopController _turnController;

    // Inputs from drive motor
    private final StatusSignal<Angle>           _drivePosition;
    private final StatusSignal<AngularVelocity> _driveVelocity;
    private final StatusSignal<Voltage>         _driveAppliedVolts;
    private final StatusSignal<Current>         _driveCurrent;

    // Queue inputs from odometry thread
    private final Queue<Double> _timestampQueue;
    private final Queue<Double> _drivePositionQueue;
    private final Queue<Double> _turnPositionQueue;

    // Connection debouncers
    private final Debouncer _driveConnectedDebouncer = new Debouncer(0.5);
    private final Debouncer _turnConnectedDebouncer  = new Debouncer(0.5);

    public ModuleIOHardware(int module)
    {
        _zeroRotation = switch (module)
        {
            case 0 -> Constants.Drive.FL_ZERO_ROTATION;
            case 1 -> Constants.Drive.FR_ZERO_ROTATION;
            case 2 -> Constants.Drive.BL_ZERO_ROTATION;
            case 3 -> Constants.Drive.BR_ZERO_ROTATION;
            default -> new Rotation2d();
        };

        _driveTalon = new TalonFX(switch (module)
        {
            case 0 -> Constants.CAN.FL_DRIVE;
            case 1 -> Constants.CAN.FR_DRIVE;
            case 2 -> Constants.CAN.BL_DRIVE;
            case 3 -> Constants.CAN.BR_DRIVE;
            default -> 0;
        });

        _turnSpark = new SparkMax(switch (module)
        {
            case 0 -> Constants.CAN.FL_TURN;
            case 1 -> Constants.CAN.FR_TURN;
            case 2 -> Constants.CAN.BL_TURN;
            case 3 -> Constants.CAN.BR_TURN;
            default -> 0;
        }, MotorType.kBrushless);

        _turnEncoder    = _turnSpark.getAbsoluteEncoder();
        _turnController = _turnSpark.getClosedLoopController();

        // Configure drive motor
        var driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode                = NeutralModeValue.Brake;
        driveConfig.Slot0.kP                               = 0.1;
        driveConfig.Slot0.kI                               = 0.0;
        driveConfig.Slot0.kD                               = 0.0;
        driveConfig.Slot0.kS                               = 0.0;
        driveConfig.Slot0.kV                               = 0.124;
        driveConfig.Feedback.SensorToMechanismRatio        = Constants.Drive.DRIVE_MOTOR_REDUCTION;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Amps.of(120.0).magnitude(); // TODO: tune this. This is the current at which wheels start to slip
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = Amps.of(-120.0).magnitude();          // TODO: tune this as well
        driveConfig.CurrentLimits.StatorCurrentLimit       = Amps.of(120.0).magnitude();       // Same as above
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted                   = Constants.Drive.DRIVE_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> _driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> _driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();

        turnConfig.inverted(Constants.Drive.TURN_INVERTED).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.Drive.TURN_MOTOR_CURRENT_LIMIT).voltageCompensation(12.0);

        turnConfig.absoluteEncoder.inverted(Constants.Drive.TURN_INVERTED).positionConversionFactor(Constants.Drive.TURN_ENCODER_POSITION_FACTOR).velocityConversionFactor(Constants.Drive.TURN_ENCODER_VELOCITY_FACTOR).averageDepth(2);

        turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).positionWrappingEnabled(true).positionWrappingInputRange(Constants.Drive.TURN_PID_MIN_INPUT, Constants.Drive.TURN_PID_MAX_INPUT)
                .pidf(Constants.Drive.TURN_KP, 0.0, Constants.Drive.TURN_KD, 0.0);

        turnConfig.signals.absoluteEncoderPositionAlwaysOn(true).absoluteEncoderPositionPeriodMs((int)(1000.0 / Constants.Drive.ODOMETRY_FREQUENCY)).absoluteEncoderVelocityAlwaysOn(true).absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        tryUntilOk(_turnSpark, 5, () -> _turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        // Create drive status signals
        _drivePosition     = _driveTalon.getPosition();
        _driveVelocity     = _driveTalon.getVelocity();
        _driveAppliedVolts = _driveTalon.getMotorVoltage();
        _driveCurrent      = _driveTalon.getStatorCurrent();

        // Create odometry queues
        _timestampQueue     = OdometryThread.getInstance().makeTimestampQueue();
        _drivePositionQueue = OdometryThread.getInstance().registerSignal(_driveTalon.getPosition());
        _turnPositionQueue  = OdometryThread.getInstance().registerSignal(_turnSpark, _turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _driveAppliedVolts, _driveCurrent);

        // Update drive inputs (Talon)
        inputs.driveConnected         = _driveConnectedDebouncer.calculate(driveStatus.isOK());
        inputs.drivePositionRad       = Units.rotationsToRadians(_drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(_driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts      = _driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps       = _driveCurrent.getValueAsDouble();

        // Update turn inputs (Spark)
        sparkStickyFault = false;

        ifOk(_turnSpark, _turnEncoder::getPosition, (value) -> inputs.turnPosition = new Rotation2d(value).minus(_zeroRotation));
        ifOk(_turnSpark, _turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(_turnSpark, new DoubleSupplier[] { _turnSpark::getAppliedOutput, _turnSpark::getBusVoltage }, (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(_turnSpark, _turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = _turnConnectedDebouncer.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps        = _timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = _drivePositionQueue.stream().mapToDouble((Double value) -> Units.rotationsToRadians(value)).toArray();
        inputs.odometryTurnPositions     = _turnPositionQueue.stream().map((Double value) -> new Rotation2d(value).minus(_zeroRotation)).toArray(Rotation2d[]::new);

        _timestampQueue.clear();
        _drivePositionQueue.clear();
        _turnPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output)
    {
        _driveTalon.setControl(_voltageRequest.withOutput(output));
    }

    @Override
    public void setTurnOpenLoop(double output)
    {
        _turnSpark.setVoltage(output);
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec)
    {
        _driveTalon.setControl(_velocityVoltageRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
    }

    @Override
    public void setTurnPosition(Rotation2d rotation)
    {
        double setpoint = MathUtil.inputModulus(rotation.plus(_zeroRotation).getRadians(), Constants.Drive.TURN_PID_MIN_INPUT, Constants.Drive.TURN_PID_MAX_INPUT);

        _turnController.setReference(setpoint, ControlType.kPosition);
    }
}
