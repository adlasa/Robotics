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
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
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
    DoubleSolenoid solenoidArm1 = new DoubleSolenoid(7, 1, 2);
    DoubleSolenoid solenoidArm2 = new DoubleSolenoid(7, 3, 4);
    DoubleSolenoid solenoidShooter = new DoubleSolenoid(7, 5, 6);
    Compressor compressor = new Compressor(1, 1);
    Timer shooterTimer = new Timer();
    Timer autoMove = new Timer();
    double autoTime;
    VisionThingy vision = new VisionThingy();
    double KP = .2;

    public double ultrasonicDistance()
    {
        SmartDashboard.putDouble("Ultrasonic Voltage", ultrasonic.getVoltage());
        return ((ultrasonic.getVoltage()) * 3.47826087) - 0.25;
    }

    public void lowerIntake()
    {
        solenoidArm1.set(DoubleSolenoid.Value.kForward);
        solenoidArm2.set(DoubleSolenoid.Value.kForward);
    }

    public void raiseIntake()
    {
        solenoidArm1.set(DoubleSolenoid.Value.kReverse);
        solenoidArm2.set(DoubleSolenoid.Value.kReverse);
    }

    public void drive()
    {
        leftDrive1.set(leftStick.getRawAxis(1));
        leftDrive2.set(leftStick.getRawAxis(1));
        leftDrive3.set(leftStick.getRawAxis(1));
        rightDrive1.set(-rightStick.getRawAxis(1));
        rightDrive2.set(-rightStick.getRawAxis(1));
        rightDrive3.set(-rightStick.getRawAxis(1));
    }

    public void intake()
    {
        intake.set(xBox.getRawAxis(2));
    }

    public void charge(double distance)
    {
        double stable = 0;
        double speed = 0;
        while(true)
        {
            speed = KP * (distance - ultrasonicDistance());
            setSpeed(speed);
            if(ultrasonicDistance() > distance - .5 || ultrasonicDistance() < distance + .5)
            {
                stable++;
                if(stable > 100)
                {
                    break;
                }
            }
        }
    }

    public void setSpeed(double speed)
    {
        leftDrive1.set(speed);
        leftDrive2.set(speed);
        leftDrive3.set(speed);
        rightDrive1.set(-speed);
        rightDrive2.set(-speed);
        rightDrive3.set(-speed);
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
        autoMove.start();
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
            if(autoMove.get() > 5)
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
            catapult1.set(0.5);
            catapult2.set(-0.5);
        }
        else
        {
            catapult1.set(0);
            catapult2.set(0);
        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl()
    {
        compressor.start();
        autoMove.reset();
        autoMove.stop();
        while(isOperatorControl() && isEnabled())
        {
            if(rightStick.getRawButton(1) && !limCatapult.get())
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
