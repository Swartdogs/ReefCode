package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants;
import frc.robot.util.Utilities;

public class ElevatorIOHardware implements ElevatorIO
{
    private final SparkFlex           _leaderSparkFlex;
    private final SparkFlex           _followerSparkFlex;
    private final RelativeEncoder     _extensionEncoder;
    private final AnalogPotentiometer _extensionPot;

    public ElevatorIOHardware()
    {
        _leaderSparkFlex   = new SparkFlex(Constants.CAN.LEAD_ELEVATOR, MotorType.kBrushless);
        _followerSparkFlex = new SparkFlex(Constants.CAN.FOLLOWER_ELEVATOR, MotorType.kBrushless);
        _extensionEncoder  = _leaderSparkFlex.getEncoder();
        _extensionPot      = new AnalogPotentiometer(Constants.AIO.EXTENSION_POT, 1,0);

        var leaderConfig = new SparkFlexConfig();

        leaderConfig.idleMode(IdleMode.kBrake).voltageCompensation(Constants.General.MOTOR_VOLTAGE).inverted(true).smartCurrentLimit(120);
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        var followerConfig = new SparkFlexConfig();

        followerConfig.idleMode(IdleMode.kBrake).voltageCompensation(Constants.General.MOTOR_VOLTAGE).follow(Constants.CAN.LEAD_ELEVATOR, true).smartCurrentLimit(120);
        followerConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        Utilities.tryUntilOk(_leaderSparkFlex, 5, () -> _leaderSparkFlex.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        Utilities.tryUntilOk(_followerSparkFlex, 5, () -> _followerSparkFlex.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        inputs.extensionPosition = _extensionPot.get();
        Utilities.ifOk(_leaderSparkFlex, _extensionEncoder::getVelocity, (value) -> inputs.extensionVelocity = value * Constants.Elevator.EXTENSION_SCALE / Constants.Elevator.EXTENSION_MOTOR_REDUCTION);

        Utilities.ifOk(_leaderSparkFlex, new DoubleSupplier[] { _leaderSparkFlex::getAppliedOutput, _leaderSparkFlex::getBusVoltage }, (values) -> inputs.leaderVolts = values[0] * values[1]);
        Utilities.ifOk(_leaderSparkFlex, _leaderSparkFlex::getOutputCurrent, (value) -> inputs.leaderCurrent = value);

        Utilities.ifOk(_followerSparkFlex, new DoubleSupplier[] { _followerSparkFlex::getAppliedOutput, _followerSparkFlex::getBusVoltage }, (values) -> inputs.followerVolts = values[0] * values[1]);
        Utilities.ifOk(_followerSparkFlex, _followerSparkFlex::getOutputCurrent, (value) -> inputs.followerCurrent = value);
    }

    @Override
    public void setVolts(double volts)
    {
        _leaderSparkFlex.setVoltage(volts);
    }
}
