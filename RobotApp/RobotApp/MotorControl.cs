﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Windows.Devices.Gpio;
using Windows.Devices.Pwm;
using Windows.Devices.Pwm.Provider;
using Windows.Foundation;
using Windows.System.Threading;
using Microsoft.IoT.DeviceCore.Pwm;
using Microsoft.IoT.DeviceHelpers;
using Microsoft.IoT.Devices.Pwm;
using Porrey.Uwp.IoT;
using Porrey.Uwp.IoT.FluentApi;
using SoftPwm = Microsoft.IoT.Devices.Pwm.SoftPwm;

namespace RobotApp
{
    /// <summary>
    ///     **** Motor Control Class ****
    ///     Handles pulse timings to motors of robot
    /// </summary>
    internal class MotorCtrl
    {
        public enum PulseMs
        {
            Stop = -1,
            Ms1 = 0,
            Ms2 = 2
        } // values selected for thread-safety
        private const int LeftPwmPin = 12;
        private const int RightPwmPin = 13;
        private const int GripperPwmPin = 18;
        private const int LeftDirectionPin1 = 23;
        private const int LeftDirectionPin2 = 22;
        private const int RightDirectionPin1 = 24;
        private const int RightDirectionPin2 = 25;
        private const int SensorPin = 4;
        private const int ActLedPin = 47; // rpi2-its-pin47, rpi-its-pin16
        private const int MaxDebs = 10;
        private static IAsyncAction _workItemThread;
        private static ulong _ticksPerMs;        
        private static GpioPin _sensorPin;
        private static GpioPin _statusLedPin;
        //public static PulseMs WaitTimeLeft = PulseMs.Stop;
        //public static PulseMs WaitTimeRight = PulseMs.Stop;
        public static int SpeedValue = 10000;
        private static long _msLastCheckTime;
        private static bool _isBlockSensed;
        private static bool _lastIsBlockSensed;
        private static bool _gpioInitialized;
        private static int[] _debounceValues;
        private static int[] _debounceCounts;
        private static int[] _debounceLast;
        private static IMotor _leftMotor;
        private static IMotor _rightMotor;
        private static Servo _servo;

        public static double SpeedMotorLeft
        {
            set { if (_leftMotor != null) _leftMotor.Speed = value; }
        }

        public static double SpeedMotorRight
        {
            set { if (_rightMotor != null) _rightMotor.Speed = value; }
        }

        public static void MotorsInit()
        {
            DebounceInit();
            GpioInit();

            _ticksPerMs = (ulong) (Stopwatch.Frequency)/1000;

            _workItemThread = ThreadPool.RunAsync(
                source =>
                {
                    // setup, ensure pins initialized
                    var mre = new ManualResetEvent(false);
                    mre.WaitOne(1000);
                    while (!_gpioInitialized)
                    {
                        CheckSystem();
                    }

                    Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int) Controllers.CtrlSpeeds.Max);

                    // settle period - to dismiss transient startup conditions, as things are starting up
                    for (var x = 0; x < 10; ++x)
                    {
                        mre.WaitOne(100);
                        _isBlockSensed = DebounceValue((int) _sensorPin.Read(), 0, 2) == 0;
                        _lastIsBlockSensed = _isBlockSensed;
                    }

                    // main motor timing loop
                    while (true)
                    {
                        mre.WaitOne(1000);
                        CheckSystem();
                    }
                }, WorkItemPriority.High);            
        }

        /// <summary>
        ///     Generate a single motor pulse wave for given servo motor (High for 1 to 2ms, duration for 20ms).
        ///     motor value denotes which moter to send pulse to.
        /// </summary>
        /// <param name="motor"></param>
        private static void PulseMotor(MotorIds motor)
        {
            // Begin pulse (setup for simple Single Speed)
            /*ulong pulseTicks = ticksPerMs;
            if (motor == MotorIds.Left)
            {
                if (waitTimeLeft == PulseMs.stop) return;
                if (waitTimeLeft == PulseMs.ms2) pulseTicks = ticksPerMs * 2;
                leftPwmPin.Write(GpioPinValue.High);
            }
            else
            {
                if (waitTimeRight == PulseMs.stop) return;
                if (waitTimeRight == PulseMs.ms2) pulseTicks = ticksPerMs * 2;
                rightPwmPin.Write(GpioPinValue.High);
            }

            // Timing
            ulong delta;
            ulong starttick = (ulong)(MainPage.stopwatch.ElapsedTicks);
            while (true)
            {
                delta = (ulong)(MainPage.stopwatch.ElapsedTicks) - starttick;
                if (delta > (20 * ticksPerMs)) break;
                if (delta > pulseTicks) break;
            }

            // End of pulse
            if (motor == MotorIds.Left) leftPwmPin.Write(GpioPinValue.Low);
            else rightPwmPin.Write(GpioPinValue.Low);*/
        }

        /// <summary>
        ///     CheckSystem - monitor for priority robot motion conditions (dead stick, or contact with object, etc.)
        /// </summary>
        private static void CheckSystem()
        {
            var msCurTime = MainPage.stopwatch.ElapsedMilliseconds;

            //--- Safety stop robot if no directions for awhile
            if ((msCurTime - Controllers.msLastDirectionTime) > 15000)
            {
                Debug.WriteLine("Safety Stop (CurTime={0}, LastDirTime={1})", msCurTime, Controllers.msLastDirectionTime);
                Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int) Controllers.CtrlSpeeds.Max);
                Controllers.FoundLocalControlsWorking = false;
                if ((msCurTime - Controllers.msLastMessageInTime) > 12000)
                {
                    NetworkCmd.NetworkInit(MainPage.serverHostName);
                }

                Controllers.XboxJoystickCheck();
            }

            //--- check on important things at a reasonable frequency
            if ((msCurTime - _msLastCheckTime) > 50)
            {
                if (_gpioInitialized)
                {
                    if (_lastIsBlockSensed != _isBlockSensed)
                    {
                        Debug.WriteLine("isBlockSensed={0}", _isBlockSensed);
                        if (_isBlockSensed)
                        {
                            BackupRobotSequence();
                            _isBlockSensed = false;
                        }
                    }
                    _lastIsBlockSensed = _isBlockSensed;
                }

                // set ACT onboard LED to indicate motor movement
                // bool stopped = (waitTimeLeft == PulseMs.stop && waitTimeRight == PulseMs.stop);
                // statusLedPin.Write(stopped ? GpioPinValue.High : GpioPinValue.Low);

                _msLastCheckTime = msCurTime;
            }
        }

        private static void DelayForTime(uint ms)
        {
            if (!_gpioInitialized) return;

            var mre = new ManualResetEvent(false);
            var stick = (ulong) MainPage.stopwatch.ElapsedTicks;
            while (true)
            {
                var delta = (ulong) (MainPage.stopwatch.ElapsedTicks) - stick;
                if (delta > (ms*_ticksPerMs)) break; // stop motion after given time
                
            }
            //Task.Delay((int) ms);
        }

        private static void BackupRobotSequence()
        {
            // stop the robot
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int) Controllers.CtrlSpeeds.Max);
            DelayForTime(200);

            // back away from the obstruction
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Backward, (int) Controllers.CtrlSpeeds.Max);
            DelayForTime(300);

            // spin 180 degress
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Right, (int) Controllers.CtrlSpeeds.Max);
            DelayForTime(800);

            // leave in stopped condition
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int) Controllers.CtrlSpeeds.Max);
        }

        private static void GpioInit()
        {
            try
            {
                var gpioController = GpioController.GetDefault();
                if (null == gpioController) return;                
                
                // Create PWM manager
                var pwmManager = new PwmProviderManager();

                // Add providers ~ pwmControllers
                //pwmManager.Providers.Add(new CustomSoftPwm(maximumSpeed));
                pwmManager.Providers.Add(new SoftPwm());
                pwmManager.Providers.Add(new PCA9685());

                // Get the well-known controller collection back
                var pwmControllers = pwmManager.GetControllersAsync().GetAwaiter().GetResult();

                // Using the first PWM controller
                var softPwmController = pwmControllers.First();
                var pwmController = pwmControllers.Last();

/*                _leftMotor = new Motor(softPwmController, LeftPwmPin, LeftDirectionPin1, LeftDirectionPin2);
                _rightMotor = new Motor(softPwmController, RightPwmPin, RightDirectionPin1, RightDirectionPin2);*/
                _leftMotor = new PwmMotor(pwmController, 8, 9, 10);
                _rightMotor = new PwmMotor(pwmController, 13, 12, 11);

                //_servo = new Servo(pwmControllers.First(), GripperPwmPin);
                
                _servo = new Servo(pwmController, 0);
                _servo.SetLimits(0.02, 0.24, 0, 180);
                _servo.Position = 30f;

                _statusLedPin = gpioController.OpenPin(ActLedPin);
                _statusLedPin.SetDriveMode(GpioPinDriveMode.Output);
                _statusLedPin.Write(GpioPinValue.Low);

                _sensorPin = gpioController.OpenPin(SensorPin);
                _sensorPin.SetDriveMode(GpioPinDriveMode.Input);
                _sensorPin.ValueChanged += (s, e) =>
                {
                    var pinValue = _sensorPin.Read();
                    _statusLedPin.Write(pinValue);
                    _isBlockSensed = (e.Edge == GpioPinEdge.RisingEdge);
                };

                _gpioInitialized = true;
            }
            catch (Exception ex)
            {
                Debug.WriteLine("ERROR: GpioInit failed - " + ex.Message);
            }
        }

        private static void DebounceInit()
        {
            _debounceValues = new int[MaxDebs];
            _debounceCounts = new int[MaxDebs];
            _debounceLast = new int[MaxDebs];
        }

        /// <summary>
        ///     DebounceValue - returns a smoothened, un-rippled, value from a run of given, possibly transient, pin values.
        ///     curValue = raw pin input value
        ///     ix = an index for a unique pin, or purpose, to locate in array
        ///     run = the maximum number of values, which signify the signal value is un-rippled or solid
        /// </summary>
        /// <param name="curValue"></param>
        /// <param name="ix"></param>
        /// <param name="run"></param>
        /// <returns></returns>
        private static int DebounceValue(int curValue, int ix, int run)
        {
            if (ix < 0 || ix > _debounceValues.Length) return 0;
            if (curValue == _debounceValues[ix])
            {
                _debounceCounts[ix] = 0;
                return curValue;
            }

            if (curValue == _debounceLast[ix]) _debounceCounts[ix] += 1;
            else _debounceCounts[ix] = 0;

            if (_debounceCounts[ix] >= run)
            {
                _debounceCounts[ix] = run;
                _debounceValues[ix] = curValue;
            }
            _debounceLast[ix] = curValue;
            return _debounceValues[ix];
        }

        private enum MotorIds
        {
            Left,
            Right
        };

        public static void ToggleGripper()
        {
            _servo.Position = _servo.Position.Equals(30f) ? 90f : 30f;
        }
    }

    public abstract class PwmMotorBase : IMotor, IDisposable
    {
        public PwmController PwmController { get; set; }
        protected readonly PwmPin PwmPin;        
        private double _speed;
        protected bool Disposed { get; set; }

        protected PwmMotorBase(PwmController pwmController, int pwmPin)
        {
            if (pwmController == null) throw new ArgumentNullException(nameof(pwmController));
            PwmController = pwmController;
            PwmPin = PwmController.OpenPin(pwmPin);
            PwmPin.Start();
        }

        /// <summary>
        ///     The speed of the motor. The sign controls the direction while the magnitude controls the speed (0 is off, 1 is full
        ///     speed).
        /// </summary>
        public double Speed
        {
            get { return _speed; }
            set
            {
                PwmPin.SetActiveDutyCyclePercentage(0);

                UpdateDirection(value);

                PwmPin.SetActiveDutyCyclePercentage(Math.Abs(value));

                _speed = value;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="value"></param>
        protected abstract void UpdateDirection(double value);

        /// <summary>
        ///     Disposes of the object releasing control the pins.
        /// </summary>
        public abstract void Dispose();
    }

    public sealed class PwmMotor : PwmMotorBase
    {
        private readonly PwmPin _directionPin1;
        private readonly PwmPin _directionPin2;

        public PwmMotor(PwmController pwmController, int pwmPin, int direction1Pin, int direction2Pin) : base(pwmController, pwmPin)
        {
            _directionPin1 = PwmController.OpenPin(direction1Pin);
            _directionPin2 = PwmController.OpenPin(direction2Pin);
            _directionPin1.Start();
            _directionPin2.Start();
        }

        protected override void UpdateDirection(double value)
        {
            if (value > 0)
            {
                _directionPin1.SetActiveDutyCyclePercentage(1);                
                _directionPin2.SetActiveDutyCyclePercentage(0);
            }
            else
            {
                _directionPin1.SetActiveDutyCyclePercentage(0);
                _directionPin2.SetActiveDutyCyclePercentage(1);
            }
        }

        public override void Dispose() => Dispose(true);

        /// <summary>
        ///     Disposes of the object releasing control the pins.
        /// </summary>
        /// <param name="disposing">Whether or not this method is called from Dispose().</param>
        private void Dispose(bool disposing)
        {
            if (Disposed) return;

            if (disposing)
            {
                PwmPin.Dispose();
                _directionPin1.Dispose();
                _directionPin2.Dispose();
            }

            Disposed = true;
        }
    }

    public sealed class Motor : PwmMotorBase
    {
        private readonly GpioPin _directionPin1;
        private readonly GpioPin _directionPin2;        

        internal Motor(PwmController pwmController, int pwmPin, int direction1Pin, int direction2Pin) : base(pwmController, pwmPin)
        {
            var gpioController = GpioController.GetDefault();
            _directionPin1 = gpioController.OpenPin(direction1Pin);
            _directionPin2 = gpioController.OpenPin(direction2Pin);
            _directionPin1.SetDriveMode(GpioPinDriveMode.Output);
            _directionPin2.SetDriveMode(GpioPinDriveMode.Output);
        }

        protected override void UpdateDirection(double value)
        {
            _directionPin1.Write(value > 0 ? GpioPinValue.High : GpioPinValue.Low);
            _directionPin2.Write(value < 0 ? GpioPinValue.High : GpioPinValue.Low);
        }

        /// <summary>
        ///     Disposes of the object releasing control the pins.
        /// </summary>
        public override void Dispose() => Dispose(true);

        /// <summary>
        ///     Disposes of the object releasing control the pins.
        /// </summary>
        /// <param name="disposing">Whether or not this method is called from Dispose().</param>
        private void Dispose(bool disposing)
        {
            if (Disposed) return;

            if (disposing)
            {
                PwmPin.Dispose();
                _directionPin1.Dispose();
                _directionPin2.Dispose();
            }

            Disposed = true;
        }
    }

    public interface IMotor
    {
        double Speed { get; set; }
    }

    public class Servo
    {
        private readonly PwmPin _pwmPin;
        private bool _limitsSet;
        private double _maxAngle;
        private double _minAngle;
        private double _offset;
        private double _position;
        private double _scale;

        internal Servo(PwmController pwmController, int pwmPin)
        {
            _position = 0.0;
            _limitsSet = false;
            _pwmPin = pwmController.OpenPin(pwmPin);
            _pwmPin.Start();
        }

        /// <summary>
        ///     The current position of the servo between the minimumAngle and maximumAngle passed to SetLimits.
        /// </summary>
        public double Position
        {
            get { return _position; }
            set
            {
                if (!_limitsSet) throw new InvalidOperationException($"You must call {nameof(SetLimits)} first.");
                if (value < _minAngle || value > _maxAngle) throw new ArgumentOutOfRangeException(nameof(value));

                _position = value;
                var dutyCyclePercentage = Math.Round(_scale * value + _offset, 2);
                _pwmPin.SetActiveDutyCyclePercentage(dutyCyclePercentage);
                Debug.WriteLine($"DutyCycle:{dutyCyclePercentage}");
            }
        }

        /// <summary>
        ///     Sets the limits of the servo.
        /// </summary>
        public void SetLimits(double minimumDutyCycle, double maximumDutyCycle, double minimumAngle, double maximumAngle)
        {
            if (minimumDutyCycle < 0) throw new ArgumentOutOfRangeException(nameof(minimumDutyCycle));
            if (maximumDutyCycle < 0) throw new ArgumentOutOfRangeException(nameof(maximumDutyCycle));
            if (minimumDutyCycle >= maximumDutyCycle) throw new ArgumentException(nameof(minimumDutyCycle));
            if (minimumAngle < 0) throw new ArgumentOutOfRangeException(nameof(minimumAngle));
            if (maximumAngle < 0) throw new ArgumentOutOfRangeException(nameof(maximumAngle));
            if (minimumAngle >= maximumAngle) throw new ArgumentException(nameof(minimumAngle));

            var pwmController = _pwmPin.Controller;
            if (!pwmController.ActualFrequency.Equals(50))
                pwmController.SetDesiredFrequency(50);

            _minAngle = minimumAngle;
            _maxAngle = maximumAngle;            

            _scale = ((maximumDutyCycle - minimumDutyCycle) /(maximumAngle - minimumAngle));
            _offset = minimumDutyCycle;

            _limitsSet = true;
        }
    }

    public class CustomSoftPwm : IPwmControllerProvider, IDisposable
    {
        private readonly double _maximumValue;
        private readonly int _pinCount;
        private Dictionary<int, ISoftPwm> _pins;
        private double _actualFrequency;

        private const int MAX_FREQUENCY = 1000;
        private const int MIN_FREQUENCY = 40;

        public CustomSoftPwm(double maximumValue)
        {
            _maximumValue = maximumValue;
            // Get GPIO
            var gpioController = GpioController.GetDefault();

            // Make sure we have it
            if (gpioController == null) { throw new DeviceNotFoundException("GPIO"); }

            // How many pins
            _pinCount = gpioController.PinCount;

            // Create pin lookup
            _pins = new Dictionary<int, ISoftPwm>(_pinCount);
        }

        public void Dispose()
        {
            // Dispose each pin
            lock (_pins)
            {
                for (int i = _pinCount - 1; i >= 0; i--)
                {
                    if (_pins.ContainsKey(i))
                    {
                        _pins[i].Pin.Dispose();
                        _pins.Remove(i);
                    }
                }
            }
            _pins = null;
        }

        public double SetDesiredFrequency(double frequency)
        {
            _actualFrequency = frequency;
            return _actualFrequency;
        }

        public void AcquirePin(int pin)
        {
            if ((pin < 0) || (pin > (_pinCount - 1))) throw new ArgumentOutOfRangeException(nameof(pin));

            lock (_pins)
            {
                if (_pins.ContainsKey(pin)) { throw new UnauthorizedAccessException(); }
                var softPwm = GpioController.GetDefault().OnPin(pin).AsExclusive().Open().AssignSoftPwm().WithPulseFrequency(_actualFrequency);
                softPwm.MaximumValue = _maximumValue;
                _pins[pin] = softPwm;
            }
        }

        public void ReleasePin(int pin)
        {
            if ((pin < 0) || (pin > (_pinCount - 1))) throw new ArgumentOutOfRangeException(nameof(pin));

            lock (_pins)
            {
                if (!_pins.ContainsKey(pin)) { throw new UnauthorizedAccessException(); }
                _pins[pin].Pin.Dispose();
                _pins.Remove(pin);
            }
        }

        public void EnablePin(int pin)
        {
            if ((pin < 0) || (pin > (_pinCount - 1))) throw new ArgumentOutOfRangeException(nameof(pin));

            lock (_pins)
            {
                if (!_pins.ContainsKey(pin)) { throw new UnauthorizedAccessException(); }
                _pins[pin].StartAsync();
            }
        }

        public void DisablePin(int pin)
        {
            if ((pin < 0) || (pin > (_pinCount - 1))) throw new ArgumentOutOfRangeException(nameof(pin));

            lock (_pins)
            {
                if (!_pins.ContainsKey(pin)) { throw new UnauthorizedAccessException(); }
                _pins[pin].StopAsync();
            }
        }

        public void SetPulseParameters(int pin, double dutyCycle, bool invertPolarity)
        {
            if ((pin < 0) || (pin > (_pinCount - 1))) throw new ArgumentOutOfRangeException(nameof(pin));

            lock (_pins)
            {
                if (!_pins.ContainsKey(pin))
                {
                    throw new UnauthorizedAccessException();
                }
                var softPin = _pins[pin];
                softPin.Value = !invertPolarity
                    ? (dutyCycle)*softPin.MaximumValue
                    : (1 - dutyCycle)*softPin.MaximumValue;                
            }

            // If duty cycle isn't zero we need to make sure updates are running
            //if ((dutyCycle != 0) && (!updater.IsStarted)) { updater.Start(); }
        }

        public double ActualFrequency => _actualFrequency;

        public double MaxFrequency => MAX_FREQUENCY;

        public double MinFrequency => MIN_FREQUENCY;

        public int PinCount => _pinCount;
    }
}