package frc.robot.subsystems.elevator;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

public class ElevatorIODashboard
{
    private static final String     MIN_HEIGHT_KEY = "Min Height";
    private static final String     EXTENSION_KEY  = "Elevator Height";
    private final NetworkTableEntry _extensionEntry;
    private double                  _minExtension  = Constants.Elevator.MIN_EXTENSION;

    public ElevatorIODashboard()
    {
        Preferences.initDouble(MIN_HEIGHT_KEY, _minExtension);

        _extensionEntry = NetworkTableInstance.getDefault().getEntry(EXTENSION_KEY);
    }

    /* CONFIGURABLE SETTINGS */

    public double getElevatorMinExtension()
    {
        _minExtension = Preferences.getDouble(MIN_HEIGHT_KEY, _minExtension);
        return _minExtension;
    }

    /* OUTPUTS */
    public void setElevatorExtension(double extension)
    {
        _extensionEntry.setDouble(extension);
    }
}
