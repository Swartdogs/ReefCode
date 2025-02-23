package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ManipulatorIOHardware implements ManipulatorIO
{
    // Hardware objects
    private final TalonSRX     _leftTalon;
    private final TalonSRX     _rightTalon;
    private final DigitalInput _lightSensorEnd;
    private final DigitalInput _lightSensorStart;

    public ManipulatorIOHardware()
    {
        _leftTalon        = new TalonSRX(Constants.CAN.MANIPULATOR_LEFT);
        _rightTalon       = new TalonSRX(Constants.CAN.MANIPULATOR_RIGHT);
        _lightSensorEnd   = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_END);
        _lightSensorStart = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_START);

        _leftTalon.setInverted(true); // Inverts upper talon so they pull/push together.
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs)
    {
        inputs.leftCurrentAmps   = _leftTalon.getStatorCurrent();
        inputs.rightCurrentAmps  = _rightTalon.getStatorCurrent();
        inputs.leftAppliedVolts  = _leftTalon.getMotorOutputVoltage();
        inputs.rightAppliedVolts = _rightTalon.getMotorOutputVoltage();

        inputs.startSensorTripped = !_lightSensorStart.get();
        inputs.endSensorTripped   = !_lightSensorEnd.get();
    }

    @Override
    public void setLeftVolts(double volts)
    {
        _leftTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    @Override
    public void setRightVolts(double volts)
    {
        _rightTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }
}
