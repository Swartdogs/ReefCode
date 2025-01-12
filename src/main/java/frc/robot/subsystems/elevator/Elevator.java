package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import static frc.robot.Constants.Elevator.*;

public class Elevator
{
    private final ElevatorIO               _io;
    private final ModuleIOInputsAutoLogged _inputs            = new ModuleIOInputsAutoLogged();
    private final PIDController            _extensionPID;
    private Double                         _extensionSetPoint = null;

    public Elevator(ElevatorIO io)
    {
        _io           = io;
        _extensionPID = new PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KP);
        _extensionPID.setTolerance(EXTENSION_TOLERANCE);
    }

    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Elevator", _inputs);

    }

    public void setExtension(double height) // height is measured in inches
    {
        _extensionSetPoint = height;

    }
}
