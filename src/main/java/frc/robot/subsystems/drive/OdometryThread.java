package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class OdometryThread
{
    private final Lock                 _signalsLock     = new ReentrantLock();
    private final List<Queue<Double>>  _phoenixQueues   = new ArrayList<>();
    private BaseStatusSignal[]         _phoenixSignals  = new BaseStatusSignal[0];
    private final List<SparkBase>      _sparks          = new ArrayList<>();
    private final List<DoubleSupplier> _sparkSignals    = new ArrayList<>();
    private final List<DoubleSupplier> _genericSignals  = new ArrayList<>();
    private final List<Queue<Double>>  _sparkQueues     = new ArrayList<>();
    private final List<Queue<Double>>  _genericQueues   = new ArrayList<>();
    private final List<Queue<Double>>  _timestampQueues = new ArrayList<>();
    private static OdometryThread      _instance        = null;
    private Notifier                   _notifier        = new Notifier(this::run);

    public static OdometryThread getInstance()
    {
        if (_instance == null)
        {
            _instance = new OdometryThread();
        }

        return _instance;
    }

    private OdometryThread()
    {
        _notifier.setName("OdometryThread");
    }

    public void start()
    {
        if (_timestampQueues.size() > 0)
        {
            _notifier.startPeriodic(1.0 / Constants.Drive.ODOMETRY_FREQUENCY);
        }
    }

    public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal)
    {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.ODOMETRY_LOCK.lock();

        try
        {
            _sparks.add(spark);
            _sparkSignals.add(signal);
            _sparkQueues.add(queue);
        }
        finally
        {
            Drive.ODOMETRY_LOCK.unlock();
        }

        return queue;
    }

    public Queue<Double> registerSignal(DoubleSupplier signal)
    {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.ODOMETRY_LOCK.lock();

        try
        {
            _genericSignals.add(signal);
            _genericQueues.add(queue);
        }
        finally
        {
            Drive.ODOMETRY_LOCK.unlock();
        }

        return queue;
    }

    public Queue<Double> registerSignal(StatusSignal<Angle> signal)
    {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);

        _signalsLock.lock();
        Drive.ODOMETRY_LOCK.lock();

        try
        {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[_phoenixSignals.length + 1];
            System.arraycopy(_phoenixSignals, 0, newSignals, 0, _phoenixSignals.length);
            newSignals[_phoenixSignals.length] = signal;
            _phoenixSignals                    = newSignals;
            _phoenixQueues.add(queue);
        }
        finally
        {
            _signalsLock.unlock();
            Drive.ODOMETRY_LOCK.unlock();
        }

        return queue;
    }

    public Queue<Double> makeTimestampQueue()
    {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.ODOMETRY_LOCK.lock();

        try
        {
            _timestampQueues.add(queue);
        }
        finally
        {
            Drive.ODOMETRY_LOCK.unlock();
        }

        return queue;
    }

    // TODO: update with TalonFX stuff
    private void run()
    {
        _signalsLock.lock();

        try
        {
            if (_phoenixSignals.length > 0)
            {
                BaseStatusSignal.refreshAll(_phoenixSignals);
            }
        }
        finally
        {
            _signalsLock.unlock();
        }

        Drive.ODOMETRY_LOCK.lock();

        try
        {
            double timestamp = RobotController.getFPGATime() / 1e6;
            double phoenixTimestamp = timestamp;
            double totalLatency = 0.0;

            for (BaseStatusSignal signal : _phoenixSignals)
            {
                totalLatency += signal.getTimestamp().getLatency();
            }

            if (_phoenixSignals.length > 0)
            {
                phoenixTimestamp -= totalLatency / _phoenixSignals.length;
            }

            double[] sparkValues = new double[_sparkSignals.size()];
            boolean  isValid     = true;

            for (int i = 0; i < _sparkSignals.size(); i++)
            {
                sparkValues[i] = _sparkSignals.get(i).getAsDouble();

                if (_sparks.get(i).getLastError() != REVLibError.kOk)
                {
                    isValid = false;
                }
            }

            if (isValid)
            {
                for (int i = 0; i < _sparkSignals.size(); i++)
                {
                    _sparkQueues.get(i).offer(sparkValues[i]);
                }

                for (int i = 0; i < _genericSignals.size(); i++)
                {
                    _genericQueues.get(i).offer(_genericSignals.get(i).getAsDouble());
                }

                for (int i = 0; i < _timestampQueues.size(); i++)
                {
                    _timestampQueues.get(i).offer((timestamp + phoenixTimestamp) / 2.0);
                }
            }
        }
        finally
        {
            Drive.ODOMETRY_LOCK.unlock();
        }
    }
}
