package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.elevator.Elevator;

public class Manipulator extends SubsystemBase
{
    private static Manipulator _instance;

    public static Manipulator getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL -> new ManipulatorIOSparkMax();
                case SIM -> new ManipulatorIOSim();
                default -> new ManipulatorIO() {};
            };

            _instance = new Manipulator(io);
        }

        return _instance;
    }

    private final ManipulatorIO                 _io;
    private final ManipulatorIOInputsAutoLogged _inputs = new ManipulatorIOInputsAutoLogged();
    private Debouncer                           _debouncer;
    private boolean                             _coralDetected;

    private Manipulator(ManipulatorIO io)
    {
        _io        = io;
        _debouncer = new Debouncer(Constants.Manipulator.DEBOUNCE_TIMER);
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        _coralDetected = (!_inputs.startSensorTripped && _inputs.endSensorTripped) || (Elevator.getInstance().getExtension() > ((Dashboard.getInstance().getElevatorL3Height() + Dashboard.getInstance().getElevatorL4Height()) / 2.0) && _inputs.endSensorTripped);

        Logger.processInputs("Manipulator", _inputs);
        Logger.recordOutput("Detected Coral", _coralDetected);
        Logger.recordOutput("Has Coral", hasCoral());
    }

    public void output()
    {
        if (Elevator.getInstance().getExtension() < Dashboard.getInstance().getElevatorL2Height())
        {
            _io.setLeftVolts(Dashboard.getInstance().getManipulatorOutputPercentSpeed() * Constants.General.MOTOR_VOLTAGE);
            _io.setRightVolts(Dashboard.getInstance().getManipulatorOutputPercentSpeed() * Dashboard.getInstance().getManipulatorL1SpeedMultiplier() * Constants.General.MOTOR_VOLTAGE);
        }
        else
        {
            _io.setVolts(Dashboard.getInstance().getManipulatorOutputPercentSpeed() * Constants.General.MOTOR_VOLTAGE);
        }
    }

    public void slowIntake()
    {
        _io.setVolts(Constants.Manipulator.SLOW_INTAKE_SPEED * Constants.General.MOTOR_VOLTAGE);
    }

    public void intake()
    {
        _io.setVolts(Dashboard.getInstance().getManipulatorIntakePercentSpeed() * Constants.General.MOTOR_VOLTAGE);
    }

    public void stop()
    {
        _io.setVolts(0);
    }

    public void setVolts(double volts)
    {
        _io.setVolts(volts);
    }

    public boolean detectedCoral()
    {
        return _coralDetected;
    }

    public boolean hasCoral()
    {
        return _debouncer.calculate(_coralDetected);
    }

    public boolean isRunning()
    {
        return _inputs.leftAppliedVolts > 0 || _inputs.rightAppliedVolts > 0;
    }

    public double getLeftOutputSpeed()
    {
        return _inputs.leftAppliedVolts / Constants.General.MOTOR_VOLTAGE;
    }

    public double getRightOutputSpeed()
    {
        return _inputs.rightAppliedVolts / Constants.General.MOTOR_VOLTAGE;
    }

    public boolean isStartSensorTripped()
    {
        return _inputs.startSensorTripped;
    }

    public boolean isEndSensorTripped()
    {
        return _inputs.endSensorTripped;
    }
}
