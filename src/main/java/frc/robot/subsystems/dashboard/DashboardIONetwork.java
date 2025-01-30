package frc.robot.subsystems.dashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashboardIONetwork implements DashboardIO
{
    public Field2d _field;

    public DashboardIONetwork()
    {
        _field = new Field2d();
        SmartDashboard.putData(_field);
    }

    @Override
    public void updateInputs(DashboardIOInputs inputs)
    {

    }

    @Override
    public void setRobotPosition(Pose2d pose)
    {
        _field.setRobotPose(pose);
    }

    @Override
    public void setPath(PathPlannerPath path)
    {
        _field.getObject("path").setPoses(path.getPathPoses());
    }

    @Override
    public void setAuto(List<PathPlannerPath> auto)
    {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (int i = 0; i < auto.size(); i++)
        {
            poses.addAll(auto.get(i).getPathPoses());
        }
        _field.getObject("auto").setPoses(poses);
    }

    @Override
    public void setMatchTime(double time)
    {

    }

    @Override
    public void setHasGamePiece(boolean hasGamePiece)
    {

    }
}
