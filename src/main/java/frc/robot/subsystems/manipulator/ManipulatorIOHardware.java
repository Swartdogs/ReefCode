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
    private final DigitalInput _lightSensor;

    public ManipulatorIOHardware()
    {
        _leftTalon   = new TalonSRX(0);
        _rightTalon  = new TalonSRX(0);
        _lightSensor = new DigitalInput(0);

        _leftTalon.setInverted(true); // Inverts upper talon so they pull/push together.
    }

    public void setVolts(double volts)
    {
        // FIXME: Use correct variable and attribute/method names.
        _rightTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
        _leftTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    public void updateInputs(ManipulatorIOInputs inputs)
    {
        inputs.leftCurrentAmps   = _leftTalon.getStatorCurrent();
        inputs.rightCurrentAmps  = _rightTalon.getStatorCurrent();
        inputs.leftAppliedVolts  = _leftTalon.getMotorOutputVoltage();
        inputs.rightAppliedVolts = _rightTalon.getMotorOutputVoltage();

        inputs.hasCoral = !_lightSensor.get();
    }
}
