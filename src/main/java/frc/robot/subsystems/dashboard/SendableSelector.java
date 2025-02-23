package frc.robot.subsystems.dashboard;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

/**
 * The {@link SendableSelector} class is a useful tool for presenting a
 * selection of options to the {@link SmartDashboard}. This class is adapted
 * from the {@link SendableChooser} class provided by the WPILibrary. The
 * adaptations made are intended to provide a way to programatically change
 * available and selected options from the robot code, instead of only allowing
 * external applications to select options.
 *
 * @param <V> The type of the values to be stored
 */
public class SendableSelector<V> implements Sendable, AutoCloseable
{
    /** The key for the default value. */
    private static final String        DEFAULT        = "default";
    /** The key for the selected option. */
    private static final String        SELECTED       = "selected";
    /** The key for the active option. */
    private static final String        ACTIVE         = "active";
    /** The key for the option array. */
    private static final String        OPTIONS        = "options";
    /** The key for the instance number. */
    private static final String        INSTANCE       = ".instance";
    private static final AtomicInteger s_instances    = new AtomicInteger();
    /** A map linking strings to the objects they represent. */
    private final Map<String, V>       _map           = new LinkedHashMap<>();
    private final int                  _instance;
    private final ReentrantLock        _mutex         = new ReentrantLock();
    private String                     _defaultChoice = "";
    private String                     _previousVal;
    private String                     _selected;
    private Consumer<V>                _listener;

    /** Instantiates a {@link SendableSelector}. */
    public SendableSelector()
    {
        _instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableSelector", _instance);
    }

    @Override
    public void close()
    {
        SendableRegistry.remove(this);
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard}
     * on the desktop, the object will appear as the given name.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void addOption(String name, V object)
    {
        _map.put(name, object);
    }

    /**
     * Adds the given object to the list of options and marks it as the default.
     * Functionally, this is very close to {@link #addOption(String, Object)} except
     * that it will use this as the default option if none other is explicitly
     * selected.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void setDefaultOption(String name, V object)
    {
        requireNonNullParam(name, "name", "setDefaultOption");

        _defaultChoice = name;
        addOption(name, object);
    }

    /**
     * Returns the selected option. If there is none selected, it will return the
     * default. If there is none selected and no default, then it will return
     * {@code null}.
     *
     * @return the option selected
     */
    public V getSelected()
    {
        _mutex.lock();

        try
        {
            if (_selected != null)
            {
                return _map.get(_selected);
            }
            else
            {
                return _map.get(_defaultChoice);
            }
        }
        finally
        {
            _mutex.unlock();
        }
    }

    public void clear()
    {
        _mutex.lock();

        _map.clear();
        _defaultChoice = "";
        _selected      = null;
        _previousVal   = null;
        _listener      = null;

        _mutex.unlock();
    }

    /**
     * Bind a listener that's called when the selected value changes. Only one
     * listener can be bound. Calling this function will replace the previous
     * listener.
     *
     * @param listener The function to call that accepts the new value
     */
    public void onChange(Consumer<V> listener)
    {
        requireNonNullParam(listener, "listener", "onChange");

        _mutex.lock();
        _listener = listener;
        _mutex.unlock();
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.setSmartDashboardType("String Chooser");
        builder.publishConstInteger(INSTANCE, _instance);
        builder.addStringProperty(DEFAULT, () -> _defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> _map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(ACTIVE, () ->
        {
            _mutex.lock();

            try
            {
                if (_selected != null)
                {
                    return _selected;
                }
                else
                {
                    return _defaultChoice;
                }
            }
            finally
            {
                _mutex.unlock();
            }
        }, null);
        builder.addStringProperty(SELECTED, () -> _selected == null ? "" : _selected, val ->
        {
            V           choice;
            Consumer<V> listener;

            _mutex.lock();

            try
            {
                _selected = val;

                if (!_selected.equals(_previousVal) && _listener != null)
                {
                    choice   = _map.get(val);
                    listener = _listener;
                }
                else
                {
                    choice   = null;
                    listener = null;
                }

                _previousVal = val;
            }
            finally
            {
                _mutex.unlock();
            }

            if (listener != null)
            {
                listener.accept(choice);
            }
        });
    }
}
