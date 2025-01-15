package frc.robot.subsystems.elevator;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO
{
    private DCMotorSim _leaderMotorSim;
    private DCMotorSim _followerMotorSim;
    private double _leaderAppliedVolts = 0.0;
    private double _followerAppliedVolts = 0.0;

    public ElevatorIOSim()
    {
        _leaderMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Elevator.ELEVATOR_GEARBOX, 0.004, Constants.Elevator.EXTENSION_MOTOR_REDUCTION), Constants.Elevator.ELEVATOR_GEARBOX);
        _followerMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Elevator.ELEVATOR_GEARBOX, 0.004, Constants.Elevator.EXTENSION_MOTOR_REDUCTION), Constants.Elevator.ELEVATOR_GEARBOX);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        inputs.leaderVolts = _leaderAppliedVolts;
        inputs.followerVolts = _followerAppliedVolts;

        inputs.leaderCurrent = _leaderMotorSim.getCurrentDrawAmps();
        inputs.followerCurrent = _followerMotorSim.getCurrentDrawAmps();

        inputs.extensionPosition = _leaderMotorSim.getAngularPositionRotations() * Constants.Elevator.EXTENSION_SCALE;
        inputs.extensionVelocity = _leaderMotorSim.getAngularVelocityRPM() * Constants.Elevator.EXTENSION_SCALE;
    }
    
    @Override 
    public void setVolts(double volts)
    {
        _leaderMotorSim.setInputVoltage(volts);
        _followerMotorSim.setInputVoltage(-volts);
        _leaderAppliedVolts = volts;
        _followerAppliedVolts = -volts;
    }
}