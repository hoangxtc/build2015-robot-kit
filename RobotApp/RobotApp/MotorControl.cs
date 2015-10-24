using System;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using Windows.Devices.Gpio;
using Windows.Devices.Pwm;
using Windows.Devices.Pwm.Provider;
using Windows.Foundation;
using Windows.System.Threading;
using Microsoft.IoT.DeviceCore.Pwm;
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
        private const int LeftPwmPin = 5;
        private const int RightPwmPin = 6;
        private const int GripperPwmPin = 19;
        private const int LeftDirectionPin1 = 13;
        private const int LeftDirectionPin2 = 12;
        private const int RightDirectionPin1 = 16;
        private const int RightDirectionPin2 = 26;
        private const int SensorPin = 13;
        private const int ActLedPin = 47; // rpi2-its-pin47, rpi-its-pin16
        private const int MaxDebs = 10;
        private static IAsyncAction _workItemThread;
        private static ulong _ticksPerMs;
        private static GpioController _gpioController;
        private static GpioPin _sensorPin;
        private static GpioPin _statusLedPin;
        public static PulseMs WaitTimeLeft = PulseMs.Stop;
        public static PulseMs WaitTimeRight = PulseMs.Stop;
        public static int SpeedValue = 10000;
        private static long _msLastCheckTime;
        private static bool _isBlockSensed;
        private static bool _lastIsBlockSensed;
        private static bool _gpioInitialized;
        private static int[] _debounceValues;
        private static int[] _debounceCounts;
        private static int[] _debounceLast;
        private static Motor _leftMotor;
        private static Motor _rightMotor;
        private static Servo _servo;

        public static int SpeedMotorLeft
        {
            set { if (_leftMotor != null) _leftMotor.Speed = value; }
        }

        public static int SpeedMotorRight
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
                        /*PulseMotor(MotorIds.Left);
                        mre.WaitOne(2);
                        PulseMotor(MotorIds.Right);
                        mre.WaitOne(2);*/

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

        private static void MoveMotorsForTime(uint ms)
        {
            if (!_gpioInitialized) return;

            var mre = new ManualResetEvent(false);
            var stick = (ulong) MainPage.stopwatch.ElapsedTicks;
            while (true)
            {
                var delta = (ulong) (MainPage.stopwatch.ElapsedTicks) - stick;
                if (delta > (ms*_ticksPerMs)) break; // stop motion after given time

                //TODO
                /*PulseMotor(MotorIds.Left);
                mre.WaitOne(2);
                PulseMotor(MotorIds.Right);
                mre.WaitOne(2);*/
            }
        }

        private static void BackupRobotSequence()
        {
            // stop the robot
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int) Controllers.CtrlSpeeds.Max);
            MoveMotorsForTime(200);

            // back away from the obstruction
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Backward, (int) Controllers.CtrlSpeeds.Max);
            MoveMotorsForTime(300);

            // spin 180 degress
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Right, (int) Controllers.CtrlSpeeds.Max);
            MoveMotorsForTime(800);

            // leave in stopped condition
            Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int) Controllers.CtrlSpeeds.Max);
        }

        private static void GpioInit()
        {
            try
            {
                _gpioController = GpioController.GetDefault();
                if (null == _gpioController) return;

                _leftMotor = new Motor(pwmChannel => _gpioController.OnPin(pwmChannel)
                    .AsExclusive()
                    .Open()
                    .AssignSoftPwm(), LeftPwmPin, LeftDirectionPin1, LeftDirectionPin2
                    //.WithValue(10000)
                    //.WithPulseFrequency(100)
                    //.WatchPulseWidthChanges((s, args) => { })
                    );
                _rightMotor = new Motor(pwmChannel => _gpioController.OnPin(pwmChannel)
                    .AsExclusive()
                    .Open()
                    .AssignSoftPwm(), RightPwmPin, RightDirectionPin1, RightDirectionPin2
                    );

                _servo = new Servo(pin =>
                {
                    // Create PWM manager
                    var pwmManager = new PwmProviderManager();

                    // Add providers ~ pwmControllers
                    pwmManager.Providers.Add(new SoftPwm());

                    // Get the well-known controller collection back
                    var pwmControllers = pwmManager.GetControllersAsync().GetAwaiter().GetResult();

                    // Using the first PWM controller
                    var controller = pwmControllers.First();

                    // Set desired frequency
                    controller.SetDesiredFrequency(50);
                    return controller.OpenPin(pin);
                }, GripperPwmPin);                

                _statusLedPin = _gpioController.OpenPin(ActLedPin);
                _statusLedPin.SetDriveMode(GpioPinDriveMode.Output);
                _statusLedPin.Write(GpioPinValue.Low);

                _sensorPin = _gpioController.OpenPin(SensorPin);
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
    }

    public class Motor : IDisposable
    {
        private readonly GpioPin _directionPin1;
        private readonly GpioPin _directionPin2;
        private readonly ISoftPwm _pwm;
        private bool _disposed;
        private double _speed;

        internal Motor(Func<int, ISoftPwm> funcPwm, int pwmPin, int direction1Pin, int direction2Pin)
        {
            _speed = 0.0;
            _disposed = false;

            var gpioController = GpioController.GetDefault();
            _directionPin1 = gpioController.OpenPin(direction1Pin);
            _directionPin2 = gpioController.OpenPin(direction2Pin);

            _directionPin1.SetDriveMode(GpioPinDriveMode.Output);
            _directionPin2.SetDriveMode(GpioPinDriveMode.Output);

            _pwm = funcPwm(pwmPin);
            _pwm.MaximumValue = 10000;
            _pwm.Start();
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
                _pwm.Value = 0.0;

                _directionPin1.Write(value > 0 ? GpioPinValue.High : GpioPinValue.Low);
                _directionPin2.Write(value > 0 ? GpioPinValue.Low : GpioPinValue.High);

                _pwm.Value = Math.Abs(value);

                _speed = value;
            }
        }

        /// <summary>
        ///     Disposes of the object releasing control the pins.
        /// </summary>
        public void Dispose() => Dispose(true);

/*
        /// <summary>
        ///     Starts the motor.
        /// </summary>
        public void Start()
        {
            _pwm.Start();
        }
*/

        /// <summary>
        ///     Stops the motor.
        /// </summary>
        public void Stop()
        {
            _pwm.Value = 0.0;
        }

        /// <summary>
        ///     Disposes of the object releasing control the pins.
        /// </summary>
        /// <param name="disposing">Whether or not this method is called from Dispose().</param>
        protected virtual void Dispose(bool disposing)
        {
            if (_disposed) return;
            if (disposing)
            {
                _directionPin1.Dispose();
                _directionPin2.Dispose();
            }

            _disposed = true;
        }
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

        internal Servo(Func<int, PwmPin> funcPwmPin, int pinNumber)
        {
            _pwmPin = funcPwmPin(pinNumber);
            _position = 0.0;
            _limitsSet = false;
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
                _pwmPin.SetActiveDutyCyclePercentage(_scale * value + _offset);
            }
        }

        /// <summary>
        ///     Sets the limits of the servo.
        /// </summary>
        public void SetLimits(int minimumDutyCycle, int maximumDutyCycle, double minimumAngle, double maximumAngle)
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
        public void Dispose()
        {
            throw new NotImplementedException();
        }

        public double SetDesiredFrequency(double frequency)
        {
            throw new NotImplementedException();
        }

        public void AcquirePin(int pin)
        {
            throw new NotImplementedException();
        }

        public void ReleasePin(int pin)
        {
            throw new NotImplementedException();
        }

        public void EnablePin(int pin)
        {
            throw new NotImplementedException();
        }

        public void DisablePin(int pin)
        {
            throw new NotImplementedException();
        }

        public void SetPulseParameters(int pin, double dutyCycle, bool invertPolarity)
        {
            throw new NotImplementedException();
        }

        public double ActualFrequency { get; }
        public double MaxFrequency { get; }
        public double MinFrequency { get; }
        public int PinCount { get; }
    }
}