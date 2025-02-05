package frc.robot.subsystems.dashboard;

import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase
{
    private final DashboardIO                  _io;
    private final DashboardIOInputsAutoLogged  _inputs     = new DashboardIOInputsAutoLogged();
    private Map<String, PathPlannerAuto>       _autoLookup = Map.of("Right_FC", new PathPlannerAuto("Right_FC"), "Left_JL", new PathPlannerAuto("Left_JL"));
    private Map<String, List<PathPlannerPath>> _pathLookup;

    public Dashboard(DashboardIO io)
    {
        _io = io;

        try
        {
            _pathLookup = Map.of("Right_FC", PathPlannerAuto.getPathGroupFromAutoFile("Right_FC"), "Left_JL", PathPlannerAuto.getPathGroupFromAutoFile("Left_JL"));

        }
        catch (Exception e)
        {
            // TODO: handle exception
        }

    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Dashboard", _inputs);
    }

    public PathPlannerAuto getAuto(String autoName)
    {
        return _autoLookup.get(autoName);
    }

    public List<PathPlannerPath> getPaths(String autoName)
    {
        return _pathLookup.get(autoName);
    }

    public void setRobotPosition(Pose2d pose)
    {
        _io.setRobotPosition(pose);
    }

    public void setPath(PathPlannerPath path)
    {
        _io.setPath(path);
    }

    public void setAuto(List<PathPlannerPath> auto)
    {
        _io.setAuto(auto);
    }
}
