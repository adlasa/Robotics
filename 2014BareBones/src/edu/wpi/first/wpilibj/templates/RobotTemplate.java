/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot
{
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    Joystick wheel = new Joystick(1);
    Joystick throttle = new Joystick(2);
    Joystick xBox = new Joystick(3);
    Victor leftDrive1 = new Victor(1);
    Victor leftDrive2 = new Victor(2);
    Victor leftDrive3 = new Victor(3);
    Victor rightDrive1 = new Victor(4);
    Victor rightDrive2 = new Victor(5);
    Victor rightDrive3 = new Victor(6);
    Victor intake = new Victor(7);
    Victor catapult1 = new Victor(8);
    Victor catapult2 = new Victor(9);
    AnalogChannel ultrasonic = new AnalogChannel(3);
    DigitalInput limCatapult = new DigitalInput(3);
    DoubleSolenoid solenoidArm = new DoubleSolenoid(7, 1, 2);
    DoubleSolenoid solenoidShooter = new DoubleSolenoid(7, 5, 6);
    Compressor compressor = new Compressor(1, 1);
    Timer shooterTimer = new Timer();
    Timer windupTimer = new Timer();
    Timer autoTime = new Timer();
    VisionThingy vision = new VisionThingy();
    //double KP = .2;

    public double ultrasonicDistance()
    {
        SmartDashboard.putDouble("Ultrasonic Voltage", ultrasonic.getVoltage());
        return ((ultrasonic.getVoltage()) * 3.47826087) - 0.25;
    }

    public void lowerIntake()
    {
        solenoidArm.set(DoubleSolenoid.Value.kForward);
    }

    public void raiseIntake()
    {
        solenoidArm.set(DoubleSolenoid.Value.kReverse);
    }

    public void drive()
    {
        if(throttle.getRawButton(2)){
            setLeftSpeed(wheel.getAxis(Joystick.AxisType.kX));
            setRightSpeed(-wheel.getAxis(Joystick.AxisType.kX));
        }else if(wheel.getAxis(Joystick.AxisType.kX) <= 0){
            setLeftSpeed(throttle.getRawAxis(1) + wheel.getAxis(Joystick.AxisType.kX));
            setRightSpeed(-throttle.getRawAxis(1));
        }else{
            setRightSpeed(-(throttle.getRawAxis(1) - wheel.getAxis(Joystick.AxisType.kX)));
            setLeftSpeed(throttle.getRawAxis(1));
        }
        
    }

    public void intake()
    {
        intake.set(xBox.getRawAxis(2)/2);
    }

    /*public void charge(double distance)
    {
        double stable = 0;
        double speed = 0;
        while(true)
        {
            speed = KP * (distance - ultrasonicDistance());
            setLeftSpeed(speed);
            setRightSpeed(-speed);
            if(ultrasonicDistance() > distance - .5 || ultrasonicDistance() < distance + .5)
            {
                stable++;
                if(stable > 100)
                {
                    break;
                }
            }
        }
    }*/

    public void setLeftSpeed(double speed)
    {
        leftDrive1.set(speed);
        leftDrive2.set(speed);
        leftDrive3.set(speed);
    }
    
    public void setRightSpeed(double speed){
        rightDrive1.set(speed);
        rightDrive2.set(speed);
        rightDrive3.set(speed);
    }

    public void autonomous()
    {
        compressor.start();
        lowerIntake();
        if(SmartDashboard.getBoolean("2 Ball Auto: "))
        {
            solenoidShooter.set(DoubleSolenoid.Value.kReverse);
            Timer.delay(1);
            solenoidShooter.set(DoubleSolenoid.Value.kForward);
            while(isAutonomous() && isEnabled())
            {
                windup();
                if(!limCatapult.get())
                {
                    intake.set(0.5);
                    Timer.delay(0.5);
                    solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                    Timer.delay(1);
                    solenoidShooter.set(DoubleSolenoid.Value.kForward);
                    break;
                }
            }
            while(isAutonomous() && isEnabled())
            {
                windup();
            }
            return;
        }
        autoTime.start();
        //charge(14);
        while(isAutonomous() && isEnabled())
        {
            if(vision.isHot)
            {
                solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                Timer.delay(1);
                solenoidShooter.set(DoubleSolenoid.Value.kForward);
                break;
            }
            if(autoTime.get() > 5)
            {
                solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                Timer.delay(1);
                solenoidShooter.set(DoubleSolenoid.Value.kForward);
                break;
            }
        }
        while(isAutonomous() && isEnabled())
        {
            windup();
        }
    }

    public void fire()
    {
        solenoidShooter.set(DoubleSolenoid.Value.kReverse);
        shooterTimer.start();
    }

    public void windup()
    {
        if(solenoidShooter.get() == DoubleSolenoid.Value.kForward && limCatapult.get())
        {
            windupTimer.start();
            catapult1.set(windupTimer.get()/2);
            catapult2.set(-(windupTimer.get())/2);
            if(windupTimer.get() > 2){
                windupTimer.stop();
            }
        }
        else
        {
            catapult1.set(0);
            catapult2.set(0);
            windupTimer.reset();
            windupTimer.stop();
        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl()
    {
        compressor.start();
        autoTime.reset();
        autoTime.stop();
        while(isOperatorControl() && isEnabled())
        {
            if(throttle.getRawButton(1) && !limCatapult.get())
            {
                fire();
            }
            else if(shooterTimer.get() > 1)
            {
                solenoidShooter.set(DoubleSolenoid.Value.kForward);
                shooterTimer.reset();
                shooterTimer.stop();
            }
            windup();
            drive();
            intake();
            if(xBox.getRawButton(5))
            {
                lowerIntake();
            }
            else if(xBox.getRawButton(6))
            {
                raiseIntake();
            }
        }
        compressor.stop();
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test()
    {
    }
}
