package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO
{
    private DCMotorSim      _leaderMotorSim;
    private DCMotorSim      _followerMotorSim;
    private MechanismRoot2d _leftStageOneRoot;
    private MechanismRoot2d _rightStageOneRoot;
    private MechanismRoot2d _leftStageTwoRoot;
    private MechanismRoot2d _rightStageTwoRoot;
    private MechanismRoot2d _carriageTopRoot;
    private MechanismRoot2d _carriageBottomRoot;
    private double          _leaderAppliedVolts   = 0.0;
    private double          _followerAppliedVolts = 0.0;

    public ElevatorIOSim()
    {
        Mechanism2d mechanism = new Mechanism2d(31, 85);

        mechanism.setBackgroundColor(new Color8Bit(Color.kBlack));

        MechanismRoot2d elevatorLeftBase  = mechanism.getRoot("Left Elevator Root", 3, 0);
        MechanismRoot2d elevatorRightBase = mechanism.getRoot("Right Elevator Root", 28, 0);
        _leftStageOneRoot   = mechanism.getRoot("Left Stage One Root", 4.5, 0);
        _rightStageOneRoot  = mechanism.getRoot("Right Stage One Root", 26.5, 0);
        _leftStageTwoRoot   = mechanism.getRoot("Left Stage Two Root", 6.0, 0);
        _rightStageTwoRoot  = mechanism.getRoot("Right Stage Two Root", 25.0, 0);
        _carriageTopRoot    = mechanism.getRoot("Carriage Top Root", 7.0, 9);
        _carriageBottomRoot = mechanism.getRoot("Carriage Bottom Root", 7.0, 0);

        elevatorLeftBase.append(new MechanismLigament2d("Left Base", 100 / 3, 90, 1, new Color8Bit(Color.kOrange)));
        elevatorRightBase.append(new MechanismLigament2d("Right Static Stage One", 100 / 3, 90, 1, new Color8Bit(Color.kOrange)));
        _leftStageOneRoot.append(new MechanismLigament2d("Left Stage One Root", 100 / 3, 90, 1, new Color8Bit(Color.kOrange)));
        _rightStageOneRoot.append(new MechanismLigament2d("Right Stage One Root", 100 / 3, 90, 1, new Color8Bit(Color.kOrange)));
        _leftStageTwoRoot.append(new MechanismLigament2d("Left Stage Two Root", 100 / 3, 90, 1, new Color8Bit(Color.kOrange)));
        _rightStageTwoRoot.append(new MechanismLigament2d("Right Stage Two Root", 100 / 3, 90, 1, new Color8Bit(Color.kOrange)));
        _carriageTopRoot.append(new MechanismLigament2d("Carriage Top", 16, 0, 1, new Color8Bit(Color.kOrange)));
        _carriageBottomRoot.append(new MechanismLigament2d("Carriage Bottom", 16, 0, 1, new Color8Bit(Color.kOrange)));
        // MechanismLigament2d leftDynamicStageOne = elevatorLeft.append(new
        // MechanismLigament2d("Left Dynamic Stage One", 0, 90, 1, new
        // Color8Bit(Color.kOrange)));
        // MechanismLigament2d rightDynamicStageOne = elevatorLeft.append(new
        // MechanismLigament2d("Right Dynamic Stage One", 0, 90, 1, new
        // Color8Bit(Color.kOrange)));
        SmartDashboard.putData("Elevator", mechanism);

        _leaderMotorSim   = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Elevator.ELEVATOR_GEARBOX, 0.004, Constants.Elevator.EXTENSION_MOTOR_REDUCTION), Constants.Elevator.ELEVATOR_GEARBOX);
        _followerMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Elevator.ELEVATOR_GEARBOX, 0.004, Constants.Elevator.EXTENSION_MOTOR_REDUCTION), Constants.Elevator.ELEVATOR_GEARBOX);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        inputs.leaderVolts   = _leaderAppliedVolts;
        inputs.followerVolts = _followerAppliedVolts;

        inputs.leaderCurrent   = _leaderMotorSim.getCurrentDrawAmps();
        inputs.followerCurrent = _followerMotorSim.getCurrentDrawAmps();

        inputs.extensionPosition = _leaderMotorSim.getAngularPositionRotations() * Constants.Elevator.EXTENSION_SCALE;
        inputs.extensionVelocity = _leaderMotorSim.getAngularVelocityRPM() * Constants.Elevator.EXTENSION_SCALE;

        _leftStageOneRoot.setPosition(4.5, inputs.extensionPosition / 3);
        _rightStageOneRoot.setPosition(26.5, inputs.extensionPosition / 3);
        _leftStageTwoRoot.setPosition(6.0, 2 * inputs.extensionPosition / 3);
        _rightStageTwoRoot.setPosition(25.0, 2 * inputs.extensionPosition / 3);
        _carriageBottomRoot.setPosition(7.0, inputs.extensionPosition - 9 * inputs.extensionPosition / Constants.Elevator.MAX_EXTENSION);
        _carriageTopRoot.setPosition(7.0, inputs.extensionPosition - 9 * inputs.extensionPosition / Constants.Elevator.MAX_EXTENSION + 9);
    }

    @Override
    public void setVolts(double volts)
    {
        _leaderMotorSim.setInputVoltage(volts);
        _followerMotorSim.setInputVoltage(-volts);
        _leaderAppliedVolts   = volts;
        _followerAppliedVolts = -volts;
    }
}
