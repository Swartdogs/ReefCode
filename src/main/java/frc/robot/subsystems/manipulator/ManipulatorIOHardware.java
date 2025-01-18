package frc.robot.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ManipulatorIOHardware implements ManipulatorIO
{

    // Hardware objects
    private final TalonSRX     _upperTalon;
    private final TalonSRX     _lowerTalon;
    private final DigitalInput _lightSensor;

    public ManipulatorIOHardware()
    {
        _upperTalon  = new TalonSRX(0);
        _lowerTalon  = new TalonSRX(0);
        _lightSensor = new DigitalInput(0);

        _upperTalon.setInverted(true); // Inverts upper talon so they pull/push together.
    }

    public void setVolts(double volts)
    {
        // FIXME: Use correct variable and attribute/method names.
        _lowerTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
        _upperTalon.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }

    public void updateInputs(ManipulatorIOInputs inputs)
    {
        inputs.topCurrentAmps     = _upperTalon.getStatorCurrent();
        inputs.bottomCurrentAmps  = _lowerTalon.getStatorCurrent();
        inputs.topAppliedVolts    = _upperTalon.getMotorOutputVoltage();
        inputs.bottomAppliedVolts = _lowerTalon.getMotorOutputVoltage();

        inputs.hasCoral = !_lightSensor.get();
    }
}
