package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.CompositeCommands;
import frc.robot.subsystems.dashboard.Dashboard;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase
{
    public enum ElevatorHeight
    {
        Stow(() -> Dashboard.getInstance().getElevatorStowHeight()), Level1(() -> Dashboard.getInstance().getElevatorL1Height()), Level2(() -> Dashboard.getInstance().getElevatorL2Height()),
        Level3(() -> Dashboard.getInstance().getElevatorL3Height()), Level4(() -> Dashboard.getInstance().getElevatorL4Height()), Hang(() -> Dashboard.getInstance().getElevatorHangHeight());

        private DoubleSupplier _heightSupplier;

        private ElevatorHeight(DoubleSupplier heightSupplier)
        {
            _heightSupplier = heightSupplier;
        }

        public double getHeight()
        {
            return _heightSupplier.getAsDouble();
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
    private final SysIdRoutine               _sysId;
    private Double                           _extensionSetpoint = null;

    private Elevator(ElevatorIO io)
    {
        _io = io;

        _extensionPID = new PIDController(Constants.Elevator.EXTENSION_KP, 0, Constants.Elevator.EXTENSION_KD);
        _extensionPID.setTolerance(Constants.Elevator.EXTENSION_TOLERANCE);

        _sysId = new SysIdRoutine(new SysIdRoutine.Config(null, Volts.of(3), null, (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())), new SysIdRoutine.Mechanism(voltage -> setVolts(voltage.in(Volts)), null, this));
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
        Logger.recordOutput("ExtensionSetpoint", _extensionSetpoint != null ? _extensionSetpoint : 0);
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
            _extensionSetpoint = Dashboard.getInstance().getElevatorStowHeight();
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

    public Command sysIdQuasistaticForward()
    {
        return _sysId.quasistatic(SysIdRoutine.Direction.kForward).until(() -> getExtension() >= Dashboard.getInstance().getElevatorL4Height());
    }

    public Command sysIdQuasistaticReverse()
    {
        return Commands.sequence(CompositeCommands.setHeight(ElevatorHeight.Level4).withTimeout(5.0), _sysId.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> getExtension() <= Dashboard.getInstance().getElevatorL1Height()));
    }

    public Command sysIdDynamicForward()
    {
        return _sysId.dynamic(SysIdRoutine.Direction.kForward).until(() -> getExtension() >= Dashboard.getInstance().getElevatorL4Height());
    }

    public Command sysIdDynamicReverse()
    {
        return Commands.sequence(CompositeCommands.setHeight(ElevatorHeight.Level4).withTimeout(5.0), _sysId.dynamic(SysIdRoutine.Direction.kReverse).until(() -> getExtension() <= Dashboard.getInstance().getElevatorL1Height()));
    }
}
