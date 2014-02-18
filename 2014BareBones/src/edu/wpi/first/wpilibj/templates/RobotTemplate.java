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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
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
    Solenoid cameraLight = new Solenoid(2, 3);
    Victor leftDrive1 = new Victor(2, 1);
    Victor leftDrive2 = new Victor(2, 2);
    Victor leftDrive3 = new Victor(2, 3);
    Victor rightDrive1 = new Victor(2, 4);
    Victor rightDrive2 = new Victor(2, 5);
    Victor rightDrive3 = new Victor(2, 6);
    Victor intake = new Victor(2, 7);
    Compressor compressor = new Compressor(2, 1, 2, 2);
    AnalogChannel ultrasonic = new AnalogChannel(2, 1);
    DoubleSolenoid solenoidArm = new DoubleSolenoid(2, 1, 2);
    Victor catapault1 = new Victor(2, 8);
    Victor catapault2 = new Victor(2, 9);
    DigitalInput limCatapult = new DigitalInput(2, 3);
    DoubleSolenoid solenoidShooter = new DoubleSolenoid(2, 5, 6);
    DriverStation driverStation = DriverStation.getInstance();
    Timer shooterTimer = new Timer();
    Timer windupTimer = new Timer();
    /*
     
     Gyro gyro = new Gyro(5, 2);
     
     
    
     Timer autoTime = new Timer();
     VisionThingy vision = new VisionThingy();
     boolean emergencyStop = false;
     Timer straightDriveTimer = new Timer();
     //double KP = .2;

     public void gyroDashboard()
     {
     SmartDashboard.putDouble("Gyro's output: ", gyro.getAngle());
     }


     */

    public void lowerIntake()
    {
        solenoidArm.set(DoubleSolenoid.Value.kForward);
    }

    public void raiseIntake()
    {
        solenoidArm.set(DoubleSolenoid.Value.kReverse);
    }

    public double ultrasonicDistance()
    {
        SmartDashboard.putDouble("Ultrasonic Distance", (ultrasonic.getVoltage() * 3.47826087) - 0.25);
        return ((ultrasonic.getVoltage()) * 3.47826087) - 0.25;
    }

    public void drive()
    {
        setLeftSpeed(-xBox.getRawAxis(2));
        setRightSpeed(xBox.getRawAxis(5));
    }

    /*public void drive()
     {
     if(throttle.getRawButton(2))
     {
     setLeftSpeed(wheel.getAxis(Joystick.AxisType.kX));
     setRightSpeed(wheel.getAxis(Joystick.AxisType.kX));
     }
     else if(wheel.getAxis(Joystick.AxisType.kX) <= 0)
     {
     setLeftSpeed(throttle.getRawAxis(1) + wheel.getAxis(Joystick.AxisType.kX));
     setRightSpeed(-throttle.getRawAxis(1));
     }
     else
     {
     setRightSpeed(-(throttle.getRawAxis(1) - wheel.getAxis(Joystick.AxisType.kX)));
     setLeftSpeed(throttle.getRawAxis(1));
     }
     }*/

    /*public void driveStraight()
     {
     setRightSpeed(-0.7);
     setLeftSpeed(0.7);
     }
    
     public void checkBattery()
     {
     if(driverStation.getBatteryVoltage() < 12)
     {
     SmartDashboard.putBoolean("Replace Battery NOW", true);
     }
     else
     {
     SmartDashboard.putBoolean("Replace Battery NOW", false);
     }
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

    public void setRightSpeed(double speed)
    {
        rightDrive1.set(speed);
        rightDrive2.set(speed);
        rightDrive3.set(speed);
    }

    public void intake()
    {
        intake.set(xBox.getRawAxis(2) / 2);
    }

    public void fire()
    {
        solenoidShooter.set(DoubleSolenoid.Value.kForward);
        shooterTimer.start();
    }

    public void windup()
    {
        if((solenoidShooter.get() == DoubleSolenoid.Value.kReverse) && !limCatapult.get())
        {
            if(windupTimer.get() == 0){
                windupTimer.start();
            } 
            catapault1.set(-(windupTimer.get() / 2));
            catapault2.set(-(windupTimer.get()) / 2);
            if(windupTimer.get() > 2)
            {
                windupTimer.stop();
            }
        }
        else
        {
            catapault1.set(0);
            catapault2.set(0);
            windupTimer.reset();
            windupTimer.stop();
        }
    }


    /*public void autonomous()
     {
     cameraLight.set(true);
     compressor.start();
     lowerIntake();
     intake.set(0.2);
     Timer.delay(0.2);
     intake.set(0);
     if(driverStation.getDigitalIn(2))
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
     straightDriveTimer.start();
     if(straightDriveTimer.get() < 5)
     {
     driveStraight();
     }
     else
     {
     setRightSpeed(0);
     setLeftSpeed(0);
     straightDriveTimer.stop();
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
     straightDriveTimer.start();
     if(straightDriveTimer.get() < 5)
     {
     driveStraight();
     }
     else
     {
     setRightSpeed(0);
     setLeftSpeed(0);
     straightDriveTimer.stop();
     }
     }
     straightDriveTimer.stop();
     }
     }

    

     /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl()
    {
        compressor.start();
        cameraLight.set(true);
        solenoidShooter.set(DoubleSolenoid.Value.kReverse);
        solenoidArm.set(DoubleSolenoid.Value.kForward);
        /*autoTime.reset();
         autoTime.stop();*/
        while(isOperatorControl() && isEnabled())
        {
            if(xBox.getRawButton(6))
            {
                lowerIntake();
            }
            else if(xBox.getRawButton(5))
            {
                raiseIntake();
            }
            intake();

            if(throttle.getRawButton(1))
            {
                fire();
            }
            else if(shooterTimer.get() > 1)
            {
                solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                shooterTimer.reset();
                shooterTimer.stop();
            }
            windup();
            ultrasonicDistance();
             //drive();
            
        }
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test()
    {
    }
}

/*if(!limCatapult.get())
             {
             catapault1.set(-Math.abs(xBox.getRawAxis(5)));
             catapault2.set(-Math.abs(xBox.getRawAxis(5))); //Only takes negative
             }else{
             catapault1.set(0);
             catapault2.set(0);
             }
             System.out.println(limCatapult.get());
             if(xBox.getRawButton(1))
             {
             solenoidShooter.set(DoubleSolenoid.Value.kReverse);
             }
             else if(xBox.getRawButton(2))
             {
             solenoidShooter.set(DoubleSolenoid.Value.kForward); //B fires
             }*/
            /*gyroDashboard();
           
             if(!emergencyStop)
             {
             windup();
             }*/
            /*intake();
             if(xBox.getRawButton(2))
             {
             emergencyStop = true;
             catapault1.set(0);
             catapault2.set(0);
             windupTimer.reset();
             windupTimer.stop();
             }
             else if(xBox.getRawButton(1))
             {
             emergencyStop = false;
             }
             }*/
