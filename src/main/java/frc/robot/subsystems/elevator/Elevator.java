package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Elevator.*;

public class Elevator extends SubsystemBase
{
    public enum ElevatorHeight
    {
        Stow(STOW_HEIGHT), Level1(L1_HEIGHT), Level2(L2_HEIGHT), Level3(L3_HEIGHT), Level4(L4_HEIGHT), Hang(HANG_HEIGHT);

        private double _height;

        private ElevatorHeight(double height)
        {
            _height = height;
        }

        public double getHeight()
        {
            return _height;
        }
    }

    private final ElevatorIO                 _io;
    private final ElevatorIOInputsAutoLogged _inputs            = new ElevatorIOInputsAutoLogged();
    private final PIDController              _extensionPID;
    private Double                           _extensionSetPoint = null;

    public Elevator(ElevatorIO io)
    {
        _io = io;

        _extensionPID = new PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD);
        _extensionPID.setTolerance(EXTENSION_TOLERANCE);
    }

    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Elevator", _inputs);

        if (_extensionSetPoint != null)
        {
            _io.setVolts(MathUtil.clamp(_extensionPID.calculate(_inputs.extensionPosition, _extensionSetPoint), -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE));

        }
        Logger.recordOutput("Has Extension Setpoint", _extensionSetPoint != null);
    }

    public void setExtension(double height) // height is measured in inches
    {
        _extensionSetPoint = height;
    }

    public void setExtension(ElevatorHeight elevatorHeight) // height is measured in inches
    {
        _extensionSetPoint = elevatorHeight.getHeight();
    }

    public void setVolts(double volts)
    {
        _io.setVolts(volts);
        _extensionSetPoint = null;
    }

    public boolean atSetpoint()
    {
        return _extensionPID.atSetpoint();
    }

    public double getExtension()
    {
        return _inputs.extensionPosition;
    }
}
