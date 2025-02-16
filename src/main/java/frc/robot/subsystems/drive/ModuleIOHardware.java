package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;

public class ModuleIOHardware implements ModuleIO
{
    private final TalonFX                       _driveMotor;
    private final SparkBase                     _turnMotor;
    private final StatusSignal<Angle>           _drivePosition;
    private final StatusSignal<AngularVelocity> _driveVelocity;
    private final StatusSignal<Voltage>         _driveAppliedVolts;
    private final StatusSignal<Current>         _driveCurrentAmps;
    private final RelativeEncoder               _turnEncoder;
    private final AnalogEncoder                 _turnPot;
    private Rotation2d                          _potOffset;

    public ModuleIOHardware(int module)
    {
        _driveMotor = new TalonFX(switch (module)
        {
            case 0 -> Constants.CAN.FL_DRIVE;
            case 1 -> Constants.CAN.FR_DRIVE;
            case 2 -> Constants.CAN.BL_DRIVE;
            case 3 -> Constants.CAN.BR_DRIVE;
            default -> 0;
        });

        _turnMotor = new SparkMax(switch (module)
        {
            case 0 -> Constants.CAN.FL_TURN;
            case 1 -> Constants.CAN.FR_TURN;
            case 2 -> Constants.CAN.BL_TURN;
            case 3 -> Constants.CAN.BR_TURN;
            default -> 0;
        }, MotorType.kBrushless);

        _turnPot = new AnalogEncoder(switch (module)
        {
            case 0 -> Constants.AIO.FL_ENCODER;
            case 1 -> Constants.AIO.FR_ENCODER;
            case 2 -> Constants.AIO.BL_ENCODER;
            case 3 -> Constants.AIO.BR_ENCODER;
            default -> 0;
        });

        _potOffset = switch (module)
        {
            case 0 -> Constants.Drive.FL_ZERO_ROTATION;
            case 1 -> Constants.Drive.FR_ZERO_ROTATION;
            case 2 -> Constants.Drive.BL_ZERO_ROTATION;
            case 3 -> Constants.Drive.BR_ZERO_ROTATION;
            default -> new Rotation2d();
        };

        // Configure drive motor
        _drivePosition     = _driveMotor.getPosition();
        _driveVelocity     = _driveMotor.getVelocity();
        _driveAppliedVolts = _driveMotor.getMotorVoltage();
        _driveCurrentAmps  = _driveMotor.getStatorCurrent();

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit       = Amps.of(120.0).magnitude();
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted                   = InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode                = NeutralModeValue.Brake;
        _driveMotor.getConfigurator().apply(driveConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, _drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, _driveVelocity, _driveAppliedVolts, _driveCurrentAmps);

        _driveMotor.optimizeBusUtilization();

        // Configure turn motor

        _turnEncoder = _turnMotor.getEncoder();

        var turnConfig = new SparkMaxConfig();

        turnConfig.inverted(Constants.Drive.TURN_INVERTED).idleMode(IdleMode.kBrake).smartCurrentLimit(Constants.Drive.TURN_MOTOR_CURRENT_LIMIT).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        turnConfig.encoder.uvwMeasurementPeriod(10).quadratureMeasurementPeriod(10).uvwAverageDepth(2).quadratureAverageDepth(2);
        turnConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderPositionPeriodMs((int)(1000.0 / Constants.Drive.ODOMETRY_FREQUENCY)).primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        _turnEncoder.setPosition(0.0);

        _turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        // Refresh all signals
        BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _driveAppliedVolts, _driveCurrentAmps);

        inputs.drivePositionRad       = _drivePosition.getValue().in(edu.wpi.first.units.Units.Radian) / Constants.Drive.DRIVE_MOTOR_REDUCTION;
        inputs.driveVelocityRadPerSec = _driveVelocity.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond) / Constants.Drive.DRIVE_MOTOR_REDUCTION;
        inputs.driveAppliedVolts      = _driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps       = _driveCurrentAmps.getValueAsDouble();

        inputs.turnAbsolutePosition  = Rotation2d.fromRotations(_turnPot.get()).minus(_potOffset);
        inputs.turnPosition          = Rotation2d.fromRotations((_turnEncoder.getPosition() / Constants.Drive.TURN_MOTOR_REDUCTION));
        inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_turnEncoder.getVelocity()) / Constants.Drive.TURN_MOTOR_REDUCTION;
        inputs.turnAppliedVolts      = _turnMotor.getAppliedOutput() * _turnMotor.getBusVoltage();
        inputs.turnCurrentAmps       = _turnMotor.getOutputCurrent();
    }

    @Override
    public void setDriveVolts(double volts)
    {
        _driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVolts(double volts)
    {
        _turnMotor.setVoltage(volts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable)
    {
        var config = new MotorOutputConfigs();
        config.Inverted    = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        _driveMotor.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable)
    {
        var config = new SparkMaxConfig();
        config.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        _turnMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setAngleOffset(Rotation2d offset)
    {
        _potOffset = offset;
    }
}
