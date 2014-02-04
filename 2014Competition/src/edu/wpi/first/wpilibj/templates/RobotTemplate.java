/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotTemplate extends SimpleRobot
{
    public final int DIGITAL_MODULE_SLOT = 2;
    public final int ANALOG_MODULE_SLOT = 1;
    public final int SOLENOID_MODULE_SLOT = 3;
    //uses 1 & 2
    Gyro gyro = new Gyro(ANALOG_MODULE_SLOT, 1);
    Joystick steerWheel = new Joystick(1);
    Joystick throttle = new Joystick(2);
    Victor leftDrive1 = new Victor(1);
    Victor leftDrive2 = new Victor(2);
    Victor leftDrive3 = new Victor(3);
    Victor rightDrive1 = new Victor(4);
    Victor rightDrive2 = new Victor(5);
    Victor rightDrive3 = new Victor(6);
    Victor intake = new Victor(7);
    Victor catapult1 = new Victor(8);
    Relay intake1 = new Relay(1);
    Relay intake2 = new Relay(2);
    Relay catapultFire = new Relay(3);
    DriverStation stupidDriverStation = DriverStation.getInstance();
    AnalogChannel ultrasonic = new AnalogChannel(3);
    JoystickButton Button1 = new JoystickButton(steerWheel, 1);
    JoystickButton Button2 = new JoystickButton(steerWheel, 2);
    JoystickButton Button3 = new JoystickButton(throttle, 1);
    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation
    final int AREA_MINIMUM = 150;
    Compressor compressor1 = new Compressor(1, 1);
    DoubleSolenoid solenoidArm1 = new DoubleSolenoid(7, 1, 2);
    DoubleSolenoid solenoidArm2 = new DoubleSolenoid(7, 3, 4);
    DoubleSolenoid solenoidShooter = new DoubleSolenoid(7, 5, 6);
    ADXL345_I2C accel = new ADXL345_I2C(DIGITAL_MODULE_SLOT, ADXL345_I2C.DataFormat_Range.k2G); // <<< not certain about the channel for this one 
    Timer time = new Timer();
    double acceleration;
    double velocity = 0;
    double distance;
    //using MaxBotix HRLV-EZ4

    public double ultrasonicDistance()
    {
        SmartDashboard.putDouble("Ultrasonic Voltage", ultrasonic.getVoltage());
        return ((ultrasonic.getVoltage()) * 3.47826087) - 0.25;
    }
    /*
     * This is where the main PID driving code has been moved, so don't panic
     */

    public void superDrive(double power, double direction)
    {
        double straightAngle;
        double pCorrection;
        double iCorrection;
        double dCorrection;
        double totalCorrection;
        double pastRate = 0;
        double kP = 0.25, kI = 1.0, kD = 0.25;
        //straightAngle = (swAdjust(steerWheel.getAxis(Joystick.AxisType.kX)) * 180) + gyro.getAngle();
        straightAngle = direction;
        pCorrection = (-gyro.getRate()) * kP;
        iCorrection = (straightAngle - gyro.getAngle()) * kI;
        dCorrection = 0 - pastRate * kD;
        totalCorrection = pCorrection + iCorrection + dCorrection;

        totalCorrection /= 10;
        if(totalCorrection > 1)
        {
            totalCorrection = 1;
        }
        else if(totalCorrection < -1)
        {
            totalCorrection = -1;
        }
        if(totalCorrection < 0)
        {
            leftSet(power * (1 - Math.abs(totalCorrection)));
            rightSet(power);
        }
        else if(totalCorrection > 0)
        {
            leftSet(power);
            rightSet(power * (1 - Math.abs(totalCorrection)));
        }
        else
        {
            bothSet(power);
        }
    }

    public void leftSet(double lDp)
    {
        leftDrive1.set(lDp);
        leftDrive2.set(lDp);
        leftDrive3.set(lDp);
        SmartDashboard.putDouble("Left Drive Power", lDp);
    }

    public void rightSet(double rDp)
    {
        rightDrive1.set(-rDp);
        rightDrive2.set(-rDp);
        rightDrive3.set(-rDp);
        SmartDashboard.putDouble("Left Drive Power", rDp);
    }

    public void bothSet(double bDp)
    {
        leftSet(bDp);
        rightSet(bDp);
    }

    public void turnSet(double tDp)
    {
        leftSet(tDp);
        rightSet(-tDp);
    }

    public void checkBattery()
    {
        if(stupidDriverStation.getBatteryVoltage() < 12)
        {
            SmartDashboard.putBoolean("Replace Battery NOW", true);
        }    
        else
        {
            SmartDashboard.putBoolean("Replace Battery NOW", false);
        }
    }

    public double batteryVoltage()
    {
        return stupidDriverStation.getBatteryVoltage();
    }

    public void robotInit()
    {
        camera = AxisCamera.getInstance();  // get an instance of the camera
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
    }

    public void fire()
    {
        //do whatever the heck the shooter team made to make it shoot thingies at the other thingies
        catapultFire.setDirection(Relay.Direction.kBoth/*change to what make it shoot things*/);
        waitSasha(1);//change to whatever it takes to fire
        catapultFire.setDirection(Relay.Direction.kBoth/*change to make it whatever it stops shooting the thingies */);
    }

    public void outputAccelData()
    {
        SmartDashboard.putDouble("X Acceleration", accel.getAcceleration(ADXL345_I2C.Axes.kX));
        SmartDashboard.putDouble("Y Acceleration", accel.getAcceleration(ADXL345_I2C.Axes.kY));
        SmartDashboard.putDouble("Z Acceleration", accel.getAcceleration(ADXL345_I2C.Axes.kZ));
    }

    //Adjust the steering whell input to normalize from -1 to 1
    double swAdjust(double i)
    {
        //increase so bottom is 0
        i += 0.945;
        //multiply so top is 2
        i *= 1.13378685;
//        put back into -1 to 1
        i -= 1.001;
        return i;
    }

    public void waitSasha(double time)
    {
        Timer wait = new Timer();
        wait.start();
        while(wait.get() < time)
        {
            System.out.println("Party time!");
            //Party Time !!!
        }
    }

    public void disabled()
    {
        System.out.println("I'm going to sleep now.");
        while(isDisabled())
        {
            checkBattery();
        }
    }

    public void computerAssistedFire()
    {
        double ultrasonicDistance = ultrasonicDistance();
        turnSet(1);
        waitSasha(0.1);
        bothSet(0);

        if(ultrasonicDistance > distance)
        {
            turnSet(-0.4);
            while(isEnabled())
            {
                if(ultrasonicDistance > distance)
                {
                    break;
                }
                distance = ultrasonicDistance;
            }
        }
        else
        {
            turnSet(0.4);
            while(isEnabled())
            {
                if(ultrasonicDistance > distance)
                {
                    break;
                }
                distance = ultrasonicDistance;
            }
        }
        bothSet(0);
        while(isEnabled())
        {
            if(ultrasonicDistance() >= 14 && ultrasonicDistance() <= 14.5)
            {
                bothSet(0);
                break;
            }
            else if(ultrasonicDistance() > 14.5)
            {
                bothSet(0.4);
            }
            else if(ultrasonicDistance() < 14)
            {
                bothSet(-0.4);
            }
            else
            {
                System.out.println("Something is going wrong I don't know what happening aaaaaaaaaahhhhhhhhhhhh");
                break;
            }
        }
        bothSet(0);
        fire();
    }

    public void autonomous()
    {
        compressor1.start();
        solenoidShooter.set(DoubleSolenoid.Value.kForward);
        VisionThingy vision = new VisionThingy();
        while(isAutonomous() && isEnabled())
        {
            superDrive(1.0, gyro.getAngle());
            if(ultrasonicDistance() >= 14 && ultrasonicDistance() <= 14.5)
            {
                bothSet(0);
                break;
            }
            else if(ultrasonicDistance() > 14.5)
            {
                bothSet(1.0);
            }
            else if(ultrasonicDistance() < 14)
            {
                bothSet(-1.0);
            }
            else
            {
                System.out.println("Something is going wrong I don't know what happening aaaaaaaaaahhhhhhhhhhhh");
                break;
            }
            fire();
            checkBattery();
        }

    }

    public void operatorControl()
    {

        //declare array for holding motor powers
        Timer heartbeat = new Timer();
        heartbeat.start();

        //main loop
        {
            while(isOperatorControl() && isEnabled())
            {
                checkBattery();
                SmartDashboard.putDouble("Heartbeat", heartbeat.get());
                heartbeat.reset();
                //output data to SmartDashboard
                SmartDashboard.putDouble("Throttle", -(throttle.getRawAxis(2)));
                SmartDashboard.putDouble("swRot", swAdjust(steerWheel.getAxis(Joystick.AxisType.kX)));

                if(Button1.get() || Button2.get())
                {
                    turnSet(swAdjust(steerWheel.getAxis(Joystick.AxisType.kX)) * 0.65);

                }
                else
                {
                    superDrive(-throttle.getRawAxis(2), ((swAdjust(steerWheel.getAxis(Joystick.AxisType.kX)) * 180) + gyro.getAngle()));
                }

                //cameraThingy();

                SmartDashboard.putDouble("Gyro", gyro.getAngle());
                // Buttons 10 and 11 for the picker-upper arms
                if(throttle.getRawButton(10))
                {
                    solenoidArm1.set(DoubleSolenoid.Value.kForward);
                    solenoidArm2.set(DoubleSolenoid.Value.kForward);
                }
                else if(throttle.getRawButton(11))
                {
                    solenoidArm1.set(DoubleSolenoid.Value.kReverse);
                    solenoidArm2.set(DoubleSolenoid.Value.kReverse);
                }
                else
                {
                    solenoidArm1.set(DoubleSolenoid.Value.kOff);
                    solenoidArm2.set(DoubleSolenoid.Value.kOff);
                }
                // firing using the trigger
                Timer timer = new Timer();
                if(throttle.getRawButton(1))
                {
                    fire();

                    solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                    timer.start();
                    if(timer.get() == 1) // 1 or 1000?
                    {
                        solenoidShooter.set(DoubleSolenoid.Value.kForward);
                        timer.reset();
                    }
                }
            }
        }
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test()
    {
        while(isTest() && isEnabled()) {
            outputAccelData();
        }
    }
}