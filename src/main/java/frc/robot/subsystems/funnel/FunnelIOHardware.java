package frc.robot.subsystems.funnel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;

public class FunnelIOHardware implements FunnelIO
{
    private final VictorSPX _funnelSolenoid;

    public FunnelIOHardware()
    {
        _funnelSolenoid = new VictorSPX(Constants.CAN.FUNNEL_SOLENOID);
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs)
    {
        inputs.funnelVolts = _funnelSolenoid.getMotorOutputVoltage();
    }

    @Override
    public void setVolts(double volts)
    {
        _funnelSolenoid.set(ControlMode.PercentOutput, volts / Constants.General.MOTOR_VOLTAGE);
    }
}
