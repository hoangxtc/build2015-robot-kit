using System;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using System.Diagnostics;
using System.Linq.Expressions;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.HumanInterfaceDevice;
using Windows.Storage;

namespace RobotApp
{
    /// <summary>
    /// **** MainPage class - controller input ****
    ///   Things in the MainPage class handle the App level startup, and App XAML level Directional inputs to the robot.
    ///   XAML sourced input controls, include screen buttons, and keyboard input
    /// </summary>
    public sealed partial class MainPage : Page
    {
        #region ----- on-screen click/touch controls -----
        private void Forward_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.Forward);
        }
        private void Left_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.Left);
        }
        private void Right_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.Right);
        }
        private void Backward_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.Backward);
        }
        private void ForwardLeft_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.ForwardLeft);
        }
        private void ForwardRight_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.ForwardRight);
        }
        private void BackwardLeft_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.BackLeft);
        }
        private void BackwardRight_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.BackRight);
        }
        private void Stop_Click(object sender, RoutedEventArgs e)
        {
            TouchDir(Controllers.CtrlCmds.Stop);
        }
        private void Status_Click(object sender, RoutedEventArgs e)
        {
            // just update the display, without affecting direction of robot.  useful for diagnosting state
            UpdateClickStatus();
        }
        private void SwitchMode_Click(object sender, RoutedEventArgs e)
        {
            SwitchRunningMode();
        }
        private void TouchDir (Controllers.CtrlCmds dir)
        {
            Controllers.FoundLocalControlsWorking = true;
            Controllers.SetRobotDirection(dir, (int)Controllers.CtrlSpeeds.Max);
            UpdateClickStatus();
        }

        private void ToggleGripper_Click(object sender, Windows.UI.Xaml.RoutedEventArgs e)
        {
            TouchButton(Controllers.CtrlCmds.ToggleGripper);
            TouchButton(Controllers.CtrlCmds.None);
        }

        private static void TouchButton(Controllers.CtrlCmds cmd)
        {
            Controllers.FoundLocalControlsWorking = true;
            Controllers.SetRobotCommand(cmd);
        }

        /// <summary>
        /// Virtual Key input handlers.  Keys directed here from XAML settings in MainPage.XAML
        /// </summary>
        private void Background_KeyDown_1(object sender, Windows.UI.Xaml.Input.KeyRoutedEventArgs e)
        {
            Debug.WriteLine("KeyDn: \"" + e.Key.ToString() + "\"");
            VKeyToRobotDirection(e.Key);
            UpdateClickStatus();
        }
        private void Background_KeyUp_1(object sender, Windows.UI.Xaml.Input.KeyRoutedEventArgs e)
        {
            VKeyToRobotDirection(Windows.System.VirtualKey.Enter);
            UpdateClickStatus();
        }
        static void VKeyToRobotDirection(Windows.System.VirtualKey vkey)
        {
            switch (vkey)
            {
                case Windows.System.VirtualKey.Down: Controllers.SetRobotDirection(Controllers.CtrlCmds.Backward,   (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.Up: Controllers.SetRobotDirection(Controllers.CtrlCmds.Forward,      (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.Left: Controllers.SetRobotDirection(Controllers.CtrlCmds.Left,       (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.Right: Controllers.SetRobotDirection(Controllers.CtrlCmds.Right,     (int)Controllers.CtrlSpeeds.Max); break;

                case Windows.System.VirtualKey.X: Controllers.SetRobotDirection(Controllers.CtrlCmds.Backward,      (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.W: Controllers.SetRobotDirection(Controllers.CtrlCmds.Forward,       (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.A: Controllers.SetRobotDirection(Controllers.CtrlCmds.Left,          (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.D: Controllers.SetRobotDirection(Controllers.CtrlCmds.Right,         (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.Z: Controllers.SetRobotDirection(Controllers.CtrlCmds.BackLeft,      (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.C: Controllers.SetRobotDirection(Controllers.CtrlCmds.BackRight,     (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.Q: Controllers.SetRobotDirection(Controllers.CtrlCmds.ForwardLeft,   (int)Controllers.CtrlSpeeds.Max); break;
                case Windows.System.VirtualKey.E: Controllers.SetRobotDirection(Controllers.CtrlCmds.ForwardRight,  (int)Controllers.CtrlSpeeds.Max); break;

                case Windows.System.VirtualKey.Enter:
                default: Controllers.SetRobotDirection(Controllers.CtrlCmds.Stop, (int)Controllers.CtrlSpeeds.Max); break;
            }
            Controllers.FoundLocalControlsWorking = true;
        }

        /// <summary>
        /// UpdateClickStatus() - fill in Connection status, and current direction State on screen after each button touch/click
        /// </summary>
        private void UpdateClickStatus()
        {
            this.CurrentState.Text = Controllers.lastSetCmd.ToString();
            if (MainPage.isRobot)
            {
                this.Connection.Text = "Robot mode";
            }
            else
            {
                if ((stopwatch.ElapsedMilliseconds - NetworkCmd.msLastSendTime) > 6000)
                {
                    this.Connection.Text = "NOT SENDING";
                }
                else
                {
                    this.Connection.Text = "OK";
                }
            }
        }
        #endregion
    }

    /// <summary>
    /// **** Controllers Class ****
    /// HID Controller devices - XBox controller
    ///   Data transfer helpers: message parsers, direction to motor value translatores, etc.
    /// </summary>
    public class Controllers
    {
        public static bool FoundLocalControlsWorking = false;

        #region ----- Xbox HID-Controller -----

        private static XboxHidController controller;
        private static int lastControllerCount = 0;
        public static async Task XboxJoystickInit()
        {
            try
            {
                string deviceSelector = HidDevice.GetDeviceSelector(0x01, 0x05);
                DeviceInformationCollection deviceInformationCollection = await DeviceInformation.FindAllAsync(deviceSelector);

                if (deviceInformationCollection.Count == 0)
                {
                    Debug.WriteLine("No Xbox360 controller found!");
                }
                lastControllerCount = deviceInformationCollection.Count;

                foreach (DeviceInformation d in deviceInformationCollection)
                {
                    Debug.WriteLine("Device ID: " + d.Id);

                    HidDevice hidDevice = await HidDevice.FromIdAsync(d.Id, FileAccessMode.Read);

                    if (hidDevice == null)
                    {
                        try
                        {
                            var deviceAccessStatus = DeviceAccessInformation.CreateFromId(d.Id).CurrentStatus;

                            if (!deviceAccessStatus.Equals(DeviceAccessStatus.Allowed)) 
                            {
                                Debug.WriteLine("DeviceAccess: " + deviceAccessStatus.ToString());
                                FoundLocalControlsWorking = true;
                            }
                        }
                        catch (Exception e)
                        {
                            Debug.WriteLine("Xbox init - " + e.Message);
                        }

                        Debug.WriteLine("Failed to connect to the controller!");
                    }

                    controller = new XboxHidController(hidDevice);
                    controller.DirectionChanged += Controller_DirectionChanged;
                    controller.ButtonChanged += Controller_ButtonChanged;
                }
            }
            catch (Exception e)
            {
                Debug.WriteLine($"XBOXJOYSTICKINIT() - {e}");
            }
        }

        public static async void XboxJoystickCheck()
        {
            string deviceSelector = HidDevice.GetDeviceSelector(0x01, 0x05);
            DeviceInformationCollection deviceInformationCollection = await DeviceInformation.FindAllAsync(deviceSelector);
            if (deviceInformationCollection.Count != lastControllerCount)
            {
                lastControllerCount = deviceInformationCollection.Count;
                await XboxJoystickInit();
            } 
        }

        private static void Controller_DirectionChanged(ControllerVector sender)
        {
            FoundLocalControlsWorking = true;
            Debug.WriteLine("Direction: " + sender.Direction + ", Magnitude: " + sender.Magnitude);
            XBoxToRobotDirection((sender.Magnitude < 2500) ? ControllerDirection.None : sender.Direction, sender.Magnitude);

            MotorCtrl.SpeedValue = sender.Magnitude;            
        }

        private static void Controller_ButtonChanged(object sender, ButtonStatus e)
        {
            FoundLocalControlsWorking = true;
            Debug.WriteLine("Button: " + e.ButtonType + ", IsActive: " + e.IsActive);
            if (e.IsActive)
            {
                XBoxToRobotCommand(e.ButtonType);
            }
        }

        private static void XBoxToRobotCommand(ButtonType buttonType)
        {
            switch (buttonType)
            {
                case ButtonType.None:
                    SetRobotCommand(CtrlCmds.None);
                    break;
                case ButtonType.A:
                case ButtonType.B:
                case ButtonType.X:
                case ButtonType.Y:
                case ButtonType.Lb:
                case ButtonType.Rb:
                case ButtonType.Back:
                case ButtonType.Start:
                case ButtonType.Lsb:
                case ButtonType.Rsb:
                    SetRobotCommand(CtrlCmds.ToggleGripper);
                    break;
                default:
                    SetRobotCommand(CtrlCmds.None);
                    break;
            }
        }

        static void XBoxToRobotDirection(ControllerDirection dir, int magnitude)
        {
            switch (dir)
            {
                case ControllerDirection.Down:
                    SetRobotDirection(CtrlCmds.Backward, magnitude);
                    break;
                case ControllerDirection.Up:
                    SetRobotDirection(CtrlCmds.Forward, magnitude);
                    break;
                case ControllerDirection.Left:
                    SetRobotDirection(CtrlCmds.Left, magnitude);
                    break;
                case ControllerDirection.Right:
                    SetRobotDirection(CtrlCmds.Right, magnitude);
                    break;
                case ControllerDirection.DownLeft:
                    SetRobotDirection(CtrlCmds.BackLeft, magnitude);
                    break;
                case ControllerDirection.DownRight:
                    SetRobotDirection(CtrlCmds.BackRight, magnitude);
                    break;
                case ControllerDirection.UpLeft:
                    SetRobotDirection(CtrlCmds.ForwardLeft, magnitude);
                    break;
                case ControllerDirection.UpRight:
                    SetRobotDirection(CtrlCmds.ForwardRight, magnitude);
                    break;
                default:
                    SetRobotDirection(CtrlCmds.Stop, (int) CtrlSpeeds.Max);
                    break;
            }
        }
        #endregion

        #region ----- general command/control helpers -----

        public enum CtrlCmds { Stop, Forward, Backward, Left, Right, ForwardLeft, ForwardRight, BackLeft, BackRight,
            None,
            ToggleGripper
        };
        public enum CtrlSpeeds { Min=0, Mid=5000, Max=10000 }

        public static long msLastDirectionTime;
        public static CtrlCmds lastSetCmd;

        public static void SetRobotDirection(CtrlCmds cmd, int speed)
        {            
            if (speed < (int) CtrlSpeeds.Min) speed = (int) CtrlSpeeds.Min;
            if (speed > (int) CtrlSpeeds.Max) speed = (int) CtrlSpeeds.Max;

            const int minus1 = -1;
            const int zero = 0;
            int speedMotorLeft;
            int speedMotorRight;
            switch (cmd)
            {
                case CtrlCmds.Forward:
                    speedMotorLeft = speed;
                    speedMotorRight = speed;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Ms2;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Ms1;*/
                    break;
                case CtrlCmds.Backward:
                    speedMotorLeft = speed * minus1;
                    speedMotorRight = speed * minus1;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Ms1;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Ms2;*/
                    break;
                case CtrlCmds.Left:
                    speedMotorLeft = speed * minus1;
                    speedMotorRight = speed;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Ms1;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Ms1;*/
                    break;
                case CtrlCmds.Right:
                    speedMotorLeft = speed;
                    speedMotorRight = speed * minus1;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Ms2;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Ms2;*/
                    break;
                case CtrlCmds.ForwardLeft:
                    speedMotorLeft = zero;
                    speedMotorRight = speed;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Stop;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Ms1;*/
                    break;
                case CtrlCmds.ForwardRight:
                    speedMotorLeft = speed;
                    speedMotorRight = zero;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Ms2;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Stop;*/
                    break;
                case CtrlCmds.BackLeft:
                    speedMotorLeft = zero;
                    speedMotorRight = speed * minus1;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Stop;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Ms2;*/
                    break;
                case CtrlCmds.BackRight:
                    speedMotorLeft = speed * minus1;
                    speedMotorRight = zero;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Ms1;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Stop;*/
                    break;
                default:
                case CtrlCmds.Stop:
                    speedMotorLeft = zero;
                    speedMotorRight = zero;
                    /*MotorCtrl.WaitTimeLeft = MotorCtrl.PulseMs.Stop;
                    MotorCtrl.WaitTimeRight = MotorCtrl.PulseMs.Stop;*/
                    break;
            }

            dumpOnDiff(cmd.ToString());

            if (!MainPage.isRobot)
            {
                String sendStr = "[" + (Convert.ToInt32(cmd)).ToString() + "]:" + cmd.ToString();
                NetworkCmd.SendCommandToRobot(sendStr);
            }
            else
            {
                MotorCtrl.SpeedMotorLeft = Convert.ToDouble(speedMotorLeft/(int) CtrlSpeeds.Max);
                MotorCtrl.SpeedMotorRight = Convert.ToDouble(speedMotorRight/(int) CtrlSpeeds.Max);
            }

            msLastDirectionTime = MainPage.stopwatch.ElapsedMilliseconds;
            lastSetCmd = cmd;
        }

        public static void SetRobotCommand(CtrlCmds cmd)
        {
            switch (cmd)
            {
                case CtrlCmds.None:
                    break;
                case CtrlCmds.ToggleGripper:
                    break;
                default:
                    break;
            }

            if (!MainPage.isRobot)
            {
                String sendStr = "[" + (Convert.ToInt32(cmd)).ToString() + "]:" + cmd.ToString();
                NetworkCmd.SendCommandToRobot(sendStr);
            }
            else
            {
                if(cmd.Equals(CtrlCmds.ToggleGripper))
                    MotorCtrl.ToggleGripper();
            }

            msLastDirectionTime = MainPage.stopwatch.ElapsedMilliseconds;
            lastSetCmd = cmd;
        }

        private static MotorCtrl.PulseMs lastWTL, lastWTR;

        private static int lastSpeed;

        static void dumpOnDiff(String title)
        {
            //TODO
            /*if ((lastWTR == MotorCtrl.WaitTimeRight) && (lastWTL == MotorCtrl.WaitTimeLeft) && (lastSpeed == MotorCtrl.SpeedValue)) return;
            Debug.WriteLine("Motors {0}: Left={1}, Right={2}, Speed={3}", title, MotorCtrl.WaitTimeLeft, MotorCtrl.WaitTimeRight, MotorCtrl.SpeedValue);
            lastWTL = MotorCtrl.WaitTimeLeft;
            lastWTR = MotorCtrl.WaitTimeRight;*/
            lastSpeed = MotorCtrl.SpeedValue;
        }

        public static long msLastMessageInTime;
        static bool lastHidCheck = false;
        public static void ParseCtrlMessage(String str)
        {
            char[] delimiterChars = { '[', ']', ':' };
            string[] words = str.Split(delimiterChars);
            if (words.Length >= 2)
            {
                int id = Convert.ToInt32(words[1]);
                if (id >= 0 && id <= 8)
                {
                    CtrlCmds cmd = (CtrlCmds)id;
                    if (FoundLocalControlsWorking)
                    {
                        if (lastHidCheck != FoundLocalControlsWorking) Debug.WriteLine("LOCAL controls found - skipping messages.");
                    }
                    else
                    {
                        if (lastHidCheck != FoundLocalControlsWorking) Debug.WriteLine("No local controls yet - using messages.");
                        SetRobotDirection(cmd, (int)CtrlSpeeds.Max);
                    }
                    lastHidCheck = FoundLocalControlsWorking;
                }
                else
                {
                    CtrlCmds cmd = (CtrlCmds)id;
                    if (FoundLocalControlsWorking)
                    {
                        if (lastHidCheck != FoundLocalControlsWorking) Debug.WriteLine("LOCAL controls found - skipping messages.");
                    }
                    else
                    {
                        if (lastHidCheck != FoundLocalControlsWorking) Debug.WriteLine("No local controls yet - using messages.");
                        SetRobotCommand(cmd);
                    }
                    
                    lastHidCheck = FoundLocalControlsWorking;
                }
            }
            msLastMessageInTime = MainPage.stopwatch.ElapsedMilliseconds;
        }

        #endregion
    }


}
