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
        _leftTalon   = new TalonSRX(Constants.CAN.MANIPULATOR_LEFT);
        _rightTalon  = new TalonSRX(Constants.CAN.MANIPULATOR_RIGHT);
        _lightSensorEnd = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_END);
        _lightSensorStart = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_START);

        _leftTalon.setInverted(true); // Inverts upper talon so they pull/push together.
    }

    public void updateInputs(ManipulatorIOInputs inputs)
    {
        inputs.leftCurrentAmps   = _leftTalon.getStatorCurrent();
        inputs.rightCurrentAmps  = _rightTalon.getStatorCurrent();
        inputs.leftAppliedVolts  = _leftTalon.getMotorOutputVoltage();
        inputs.rightAppliedVolts = _rightTalon.getMotorOutputVoltage();

        inputs.hasCoral = !_lightSensorEnd.get();
    }

    public void setVolts(double volts)
    {
        _rightTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
        _leftTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }
}
