package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.dashboard.DashboardIO.DashboardIOInputs;

public class Dashboard extends SubsystemBase
{
    private final DashboardIO                 _io;
    private final DashboardIOInputsAutoLogged _inputs = new DashboardIOInputsAutoLogged();

    public Dashboard(DashboardIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Dashboard", _inputs);
    }

    public void setRobotPosition(Pose2d pose)
    {
        _io.setRobotPosition(pose);
    }
}
