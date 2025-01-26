package frc.robot.subsystems.dashboard;

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
    public void setMatchTime(double time)
    {

    }

    @Override
    public void setHasGamePiece(boolean hasGamePiece)
    {

    }
}
