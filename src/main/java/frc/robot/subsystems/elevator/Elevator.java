package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
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

    private static Elevator _instance;

    public static Elevator getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL -> new ElevatorIOHardware();
                case SIM -> new ElevatorIOSim();
                default -> new ElevatorIO() {};
            };

            _instance = new Elevator(io);
        }

        return _instance;
    }

    private final ElevatorIO                 _io;
    private final ElevatorIOInputsAutoLogged _inputs            = new ElevatorIOInputsAutoLogged();
    private final PIDController              _extensionPID;
    private Double                           _extensionSetpoint = null;
    private final Alert                      _potAlert;
    private double                           _lastPosition      = 0.0; // Last 20 milisecond elevator position
    private final SysIdRoutine               _sysId;

    private Elevator(ElevatorIO io)
    {
        _io = io;

        _extensionPID = new PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD);
        _extensionPID.setTolerance(EXTENSION_TOLERANCE);

        _potAlert     = new Alert("Potentiometer has been disconnected", AlertType.kError);
        _sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(voltage -> setVolts(voltage.magnitude()), log ->
                      {
                          log.motor("Elevator").voltage(Volts.ofBaseUnits(_inputs.leaderVolts)).linearPosition(Meters.ofBaseUnits(_inputs.extensionPosition)).linearVelocity(MetersPerSecond.ofBaseUnits(_inputs.extensionVelocity));
                      }, this));
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Elevator", _inputs);

        if (_extensionSetpoint != null)
        {
            _io.setVolts(MathUtil.clamp(_extensionPID.calculate(_inputs.extensionPosition, _extensionSetpoint) + Constants.Elevator.ELEVATOR_FEED_FORWARD, -Constants.General.MOTOR_VOLTAGE / 5, Constants.General.MOTOR_VOLTAGE));
            Logger.recordOutput("Setpoint", _extensionSetpoint);
        }

        Logger.recordOutput("Has Extension Setpoint", _extensionSetpoint != null);

        if (_inputs.leaderVolts != 0 && _lastPosition == _inputs.extensionPosition) // if voltage is not 0 and last position does not change then error
        {
            _potAlert.set(true);
        }

        _lastPosition = _inputs.extensionPosition;
    }

    public void setExtension(double height) // height is measured in inches
    {
        _extensionSetpoint = height;
    }

    public void setExtension(ElevatorHeight elevatorHeight) // height is measured in inches
    {
        _extensionSetpoint = elevatorHeight.getHeight();
    }

    public void setVolts(double volts)
    {
        _io.setVolts(Math.min(volts + Constants.Elevator.ELEVATOR_FEED_FORWARD, 12.0));
        _extensionSetpoint = null;
    }

    public void modifySetpoint(double modification)
    {
        if (_extensionSetpoint == null)
        {
            _extensionSetpoint = Constants.Elevator.STOW_HEIGHT;
        }

        _extensionSetpoint += modification;
    }

    public boolean atSetpoint()
    {
        return _extensionPID.atSetpoint() || _extensionSetpoint == null;
    }

    public double getExtension()
    {
        return _inputs.extensionPosition;
    }

    public Double getSetpoint()
    {
        return _extensionSetpoint;
    }

    public void stop()
    {
        _extensionSetpoint = null;
        _io.setVolts(0);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return _sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return _sysId.dynamic(direction);
    }
}
