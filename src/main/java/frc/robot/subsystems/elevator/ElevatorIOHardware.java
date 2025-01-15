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

import static frc.robot.util.SparkUtil.*;

public class ElevatorIOHardware implements ElevatorIO
{
    private final SparkFlex        _leaderMotor;
    private final SparkFlex        _followerMotor;
    private final RelativeEncoder _extensionEncoder; 
    private final AnalogPotentiometer _extensionPot;

    public ElevatorIOHardware()
    {
        _leaderMotor      = new SparkFlex(Constants.CAN.LEAD_ELEVATOR, MotorType.kBrushless);
        _followerMotor    = new SparkFlex(Constants.CAN.FOLLOWER_ELEVATOR, MotorType.kBrushless);
        _extensionEncoder = _leaderMotor.getEncoder();
        _extensionPot = new AnalogPotentiometer(Constants.AIO.EXTENSION_POT, Constants.Elevator.EXTENSION_SCALE, -Constants.Elevator.EXTENSION_OFFSET);

        var leaderConfig = new SparkFlexConfig();

        leaderConfig.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
        leaderConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        var followerConfig = new SparkFlexConfig();

        followerConfig.idleMode(IdleMode.kBrake).follow(Constants.CAN.LEAD_ELEVATOR).inverted(true).voltageCompensation(12);
        followerConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(0, Constants.Elevator.MAX_EXTENSION);

        tryUntilOk(_leaderMotor, 5, () -> _leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(_followerMotor, 5, () -> _followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        inputs.extensionPosition = _extensionPot.get();
        ifOk(_leaderMotor, _extensionEncoder::getVelocity, (value)-> inputs.extensionVelocity = value * Constants.Elevator.EXTENSION_SCALE/Constants.Elevator.EXTENSION_MOTOR_REDUCTION);
        ifOk(_leaderMotor, new DoubleSupplier[] { _leaderMotor::getAppliedOutput, _leaderMotor::getBusVoltage }, (values) -> inputs.leaderVolts = values[0] * values[1]);
        ifOk(_leaderMotor, _leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrent = value);

       
        ifOk(_followerMotor, new DoubleSupplier[] { _followerMotor::getAppliedOutput, _leaderMotor::getBusVoltage }, (values) -> inputs.leaderVolts = values[0] * values[1]);
        ifOk(_followerMotor, _leaderMotor::getOutputCurrent, (value) -> inputs.leaderCurrent = value);
    }

    @Override
    public void setVolts(double volts)
    {
        _leaderMotor.setVoltage(volts);
    }
}
