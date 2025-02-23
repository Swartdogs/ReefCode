package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.Dashboard.DashboardSetting;
import frc.robot.subsystems.funnel.Funnel;

public class FunnelCommands
{
    private FunnelCommands()
    {
    }

    public static Command drop()
    {
        return Commands.defer(
                () -> Commands
                        .startEnd(() -> Funnel.getInstance().setVolts(Dashboard.getInstance().getSetting(DashboardSetting.FunnelRetractSpeed) * Constants.General.MOTOR_VOLTAGE), () -> Funnel.getInstance().setVolts(0), Funnel.getInstance())
                        .withTimeout(Dashboard.getInstance().getSetting(DashboardSetting.FunnelRetractTime)),
                Set.of(Funnel.getInstance())
        );
    }
}
