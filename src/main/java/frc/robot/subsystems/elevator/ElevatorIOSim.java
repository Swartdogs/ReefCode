package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO
{
    private DCMotorSim      _leaderMotorSim;
    private DCMotorSim      _followerMotorSim;
    private MechanismRoot2d _leftStageOneRoot;
    private MechanismRoot2d _rightStageOneRoot;
    private MechanismRoot2d _leftStageTwoRoot;
    private MechanismRoot2d _rightStageTwoRoot;
    private MechanismRoot2d _carriageBottomRoot;
    private double          _leaderAppliedVolts   = 0.0;
    private double          _followerAppliedVolts = 0.0;
    private ElevatorSim     _elevatorSim;

    public ElevatorIOSim()
    {
        Mechanism2d mechanism = new Mechanism2d(31, 80);

        mechanism.setBackgroundColor(new Color8Bit(Color.kBlack));

        MechanismRoot2d elevatorLeftBase  = mechanism.getRoot("Left Elevator Root", 3, 0);
        MechanismRoot2d elevatorRightBase = mechanism.getRoot("Right Elevator Root", 28, 0);
        _leftStageOneRoot   = mechanism.getRoot("Left Stage One Root", 4.5, 0);
        _rightStageOneRoot  = mechanism.getRoot("Right Stage One Root", 26.5, 0);
        _leftStageTwoRoot   = mechanism.getRoot("Left Stage Two Root", 6.0, 0);
        _rightStageTwoRoot  = mechanism.getRoot("Right Stage Two Root", 25.0, 0);
        _carriageBottomRoot = mechanism.getRoot("Carriage Bottom Root", 7.0, 0);

        var carriageBottom = new MechanismLigament2d("Carriage Bottom", 17, 0, 10, new Color8Bit(Color.kOrange));
        var carriageRight  = new MechanismLigament2d("Carriage Right", 9, 90, 10, new Color8Bit(Color.kOrange));
        var carriageTop    = new MechanismLigament2d("Carriage Top", 17, 90, 10, new Color8Bit(Color.kOrange));
        var carriageLeft   = new MechanismLigament2d("Carriage Left", 9, 90, 10, new Color8Bit(Color.kOrange));

        elevatorLeftBase.append(new MechanismLigament2d("Left Base", 100 / 3, 90, 10, new Color8Bit(Color.kOrange)));
        elevatorRightBase.append(new MechanismLigament2d("Right Static Stage One", 100 / 3, 90, 10, new Color8Bit(Color.kOrange)));
        _leftStageOneRoot.append(new MechanismLigament2d("Left Stage One Root", 100 / 3, 90, 10, new Color8Bit(Color.kOrange)));
        _rightStageOneRoot.append(new MechanismLigament2d("Right Stage One Root", 100 / 3, 90, 10, new Color8Bit(Color.kOrange)));
        _leftStageTwoRoot.append(new MechanismLigament2d("Left Stage Two Root", 100 / 3, 90, 10, new Color8Bit(Color.kOrange)));
        _rightStageTwoRoot.append(new MechanismLigament2d("Right Stage Two Root", 100 / 3, 90, 10, new Color8Bit(Color.kOrange)));
        _carriageBottomRoot.append(carriageBottom);
        carriageBottom.append(carriageRight);
        carriageRight.append(carriageTop);
        carriageTop.append(carriageLeft);

        _elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(Constants.Elevator.ELEVATOR_GEARBOX, Constants.Elevator.ELEVATOR_MASS, Constants.Elevator.ELEVATOR_DRUM_RADIUS, Constants.Elevator.EXTENSION_MOTOR_REDUCTION),
                Constants.Elevator.ELEVATOR_GEARBOX, 0.0, Units.inchesToMeters(Constants.Elevator.MAX_EXTENSION), true, 0.0
        );
        SmartDashboard.putData("Elevator", mechanism);

        // _leaderMotorSim = new
        // DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Elevator.ELEVATOR_GEARBOX,
        // 0.004, Constants.Elevator.EXTENSION_MOTOR_REDUCTION),
        // Constants.Elevator.ELEVATOR_GEARBOX);
        // _followerMotorSim = new
        // DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Elevator.ELEVATOR_GEARBOX,
        // 0.004, Constants.Elevator.EXTENSION_MOTOR_REDUCTION),
        // Constants.Elevator.ELEVATOR_GEARBOX);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        // _leaderMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        // _followerMotorSim.update(Constants.General.LOOP_PERIOD_SECS);

        inputs.leaderVolts   = _leaderAppliedVolts;
        inputs.followerVolts = _followerAppliedVolts;

        inputs.leaderCurrent   = _elevatorSim.getCurrentDrawAmps();
        inputs.followerCurrent = _elevatorSim.getCurrentDrawAmps();

        inputs.extensionPosition = Units.metersToInches(_elevatorSim.getPositionMeters());
        inputs.extensionVelocity = Units.metersToInches(_elevatorSim.getVelocityMetersPerSecond());

        _leftStageOneRoot.setPosition(4.5, MathUtil.clamp(inputs.extensionPosition - 2 * (Constants.Elevator.MAX_EXTENSION / 3.0), 0.0, Constants.Elevator.MAX_EXTENSION / 3.0));
        _rightStageOneRoot.setPosition(26.5, MathUtil.clamp(inputs.extensionPosition - 2 * (Constants.Elevator.MAX_EXTENSION / 3.0), 0.0, Constants.Elevator.MAX_EXTENSION / 3));
        _leftStageTwoRoot.setPosition(6.0, MathUtil.clamp(inputs.extensionPosition - Constants.Elevator.MAX_EXTENSION / 3.0, 0.0, Constants.Elevator.MAX_EXTENSION / 3 * 2));
        _rightStageTwoRoot.setPosition(25.0, MathUtil.clamp(inputs.extensionPosition - Constants.Elevator.MAX_EXTENSION / 3.0, 0.0, Constants.Elevator.MAX_EXTENSION / 3 * 2));
        _carriageBottomRoot.setPosition(7.0, MathUtil.clamp(inputs.extensionPosition, 0.0, Constants.Elevator.MAX_EXTENSION));

        _elevatorSim.update(Constants.General.LOOP_PERIOD_SECS);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(_elevatorSim.getCurrentDrawAmps()));

    }

    @Override
    public void setVolts(double volts)
    {
        _elevatorSim.setInputVoltage(volts);
        _leaderAppliedVolts   = volts;
        _followerAppliedVolts = -volts;
    }
}
