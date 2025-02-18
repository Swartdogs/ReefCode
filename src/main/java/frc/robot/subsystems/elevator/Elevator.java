package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
    private final ProfiledPIDController      _extensionPID;
    private Double                           _extensionSetPoint = null;
    private final Alert                      _potAlert;
    private double                           _lastPosition      = 0.0; // Last 20 milisecond elevator position

    public Elevator(ElevatorIO io)
    {
        _io = io;

        _extensionPID = new ProfiledPIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        _extensionPID.reset(Constants.Elevator.SCALED_MIN);

        _potAlert = new Alert("Potentiometer has been disconnected", AlertType.kError);
    }

    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Elevator", _inputs);

        if (_extensionSetPoint != null)
        {
            _io.setVolts(MathUtil.clamp(_extensionPID.calculate(_inputs.extensionPosition) + Constants.Elevator.ELEVATOR_FEED_FORWARD, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE));
        }

        Logger.recordOutput("Has Extension Setpoint", _extensionSetPoint != null);

        if (_inputs.leaderVolts != 0 && _lastPosition == _inputs.extensionPosition) // if voltage is not 0 and last position does not change then error
        {
            _potAlert.set(true);
        }

        _lastPosition = _inputs.extensionPosition;

        Logger.recordOutput("ElevatorSetpointPosition", _extensionPID.getSetpoint().position);
        Logger.recordOutput("ElevatorSetpointVelocity", _extensionPID.getSetpoint().velocity);
    }

    public void setExtension(double height) // height is measured in inches
    {
        _extensionSetPoint = height;
        _extensionPID.setGoal(_extensionSetPoint);
    }

    public void setExtension(ElevatorHeight elevatorHeight) // height is measured in inches
    {
        _extensionSetPoint = elevatorHeight.getHeight();
        _extensionPID.setGoal(_extensionSetPoint);
    }

    public void setVolts(double volts)
    {
        _io.setVolts(Math.min(volts + Constants.Elevator.ELEVATOR_FEED_FORWARD, 12.0));
        _extensionSetPoint = null;
    }

    public void modifySetpoint(double modification)
    {
        _extensionSetPoint += modification;
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
