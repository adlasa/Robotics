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
    Victor leftDrive1 = new Victor(1);
    Victor leftDrive2 = new Victor(2);
    Victor leftDrive3 = new Victor(3);
    Victor rightDrive1 = new Victor(4);
    Victor rightDrive2 = new Victor(5);
    Victor rightDrive3 = new Victor(6);
    Victor intake = new Victor(7);
    Compressor compressor = new Compressor(1, 2);
    AnalogChannel ultrasonic = new AnalogChannel(1);
    DoubleSolenoid solenoidArm = new DoubleSolenoid(2, 1, 2);
    Victor catapault1 = new Victor(8);
    Victor catapault2 = new Victor(9);
    DigitalInput limCatapult = new DigitalInput(3);
    DoubleSolenoid solenoidShooter = new DoubleSolenoid(2, 5, 6);
    DriverStation driverStation = DriverStation.getInstance();
    boolean windupFlag = false;
    Timer shooterTimer = new Timer();
    Timer windupTimer = new Timer();
    Timer autoTime = new Timer();
    VisionThingy vision = new VisionThingy();
    Timer straightDriveTimer = new Timer();
    /*
     
     Gyro gyro = new Gyro(5, 2);
     
     
    
     
    
     boolean emergencyStop = false;
     
     //double KP = .2;

     public void gyroDashboard()
     {
     SmartDashboard.putDouble("Gyro's output: ", gyro.getAngle());
     }


     */

    double swAdjust(double i)
    {
        //increase so bottom is 0
        i += 0.945;
        //multiply so top is 2
        i *= 1.13378685;
        //put back into -1 to 1
        i -= 1.001;
        return i;
    }

    public void lowerIntake()
    {
        solenoidArm.set(DoubleSolenoid.Value.kReverse);
    }

    public void raiseIntake()
    {
        solenoidArm.set(DoubleSolenoid.Value.kForward);
    }

    public double ultrasonicDistance()
    {
        SmartDashboard.putDouble("Ultrasonic Distance", (ultrasonic.getVoltage() * 3.47826087) - 0.25);
        return ((ultrasonic.getVoltage()) * 3.47826087) - 0.25;
    }

    /*public void drive()
     {
     setLeftSpeed(-xBox.getRawAxis(2));
     setRightSpeed(xBox.getRawAxis(5));
     }*/
    public void drive()
    {
        if(throttle.getRawButton(1))
        {
            setLeftSpeed(swAdjust(-wheel.getAxis(Joystick.AxisType.kX)));
            setRightSpeed(swAdjust(-wheel.getAxis(Joystick.AxisType.kX)));
        }
        else if(wheel.getAxis(Joystick.AxisType.kX) <= 0)
        {
            setLeftSpeed(throttle.getRawAxis(2) * (1 + swAdjust(wheel.getAxis(Joystick.AxisType.kX))));
            setRightSpeed(-throttle.getRawAxis(2));
        }
        else
        {
            setRightSpeed(-(throttle.getRawAxis(2) * (1 - swAdjust(wheel.getAxis(Joystick.AxisType.kX)))));
            setLeftSpeed(throttle.getRawAxis(2));
        }
    }
    /*public void drive(){
        
     setLeftSpeed(xBox.getRawAxis(2));
     setRightSpeed(-xBox.getRawAxis(5));
     }*/

    public void driveStraight(double speed)
    {
        setRightSpeed(-speed);
        setLeftSpeed(speed);
    }

    /* public void checkBattery()
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
    /*public void superDrive(double power, double direction)
    {
        double pCorrection;
        double iCorrection;
        double totalCorrection;
        final double kP = 0.25, kI = 1.0;
        //d was 0.25
        direction = (direction) + gyro.getAngle();
        //straightAngle = direction;
        pCorrection = (-gyro.getRate()) * kP;
        iCorrection = (direction - gyro.getAngle()) * kI;
        totalCorrection = pCorrection + iCorrection;

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
            setLeftSpeed(power * (1 - Math.abs(totalCorrection)));
            setRightSpeed(-power);
        }
        else if(totalCorrection > 0)
        {
            setLeftSpeed(power);
            setRightSpeed(-power * (1 - Math.abs(totalCorrection)));
        }
        else
        {
            setLeftSpeed(power);
            setRightSpeed(-power);
        }
    }

    public void wallDistance(final double distance)
    {
        final double kP = -0.125;

        double cP;

        double power;

        double ultrasonicDistance;

        double straightAngle;
        double pCorrection;
        double iCorrection;
        double totalCorrection;
        final double GkP = 0.25, GkI = 1.0;
        straightAngle = gyro.getAngle();

        while(isEnabled())
        {
            ultrasonicDistance = ultrasonicDistance();
            cP = (distance - ultrasonicDistance) * kP;
            power = 1.0 * cP;

            pCorrection = (-gyro.getRate()) * GkP;
            iCorrection = (straightAngle - gyro.getAngle()) * GkI;
            totalCorrection = pCorrection + iCorrection;

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
                setLeftSpeed(-power * (1 - Math.abs(totalCorrection)));
                setRightSpeed(-power);
            }
            else if(totalCorrection > 0)
            {
                setLeftSpeed(-power);
                setRightSpeed(-power * (1 - Math.abs(totalCorrection)));
            }
            else
            {
                bothSet(-power);
            }
            if(Math.abs(ultrasonicDistance - distance) < 0.25)
            {
                break;
            }
            bothSet(0);
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

    public void windup()
    {
        if((solenoidShooter.get() == DoubleSolenoid.Value.kReverse) && !limCatapult.get())
        {
            if(windupTimer.get() == 0)
            {
                windupTimer.start();
            }
            catapault1.set(-(windupTimer.get() / 5)); //it was 4
            catapault2.set(-(windupTimer.get()) / 5);
            if(windupTimer.get() > 5)
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
            windupFlag = false;
        }
    }

    public void autonomous()
    {
        cameraLight.set(true);
        compressor.start();
        lowerIntake();
        if(!driverStation.getDigitalIn(2))
        {
            autoTime.start();
            driveStraight(0.4);
            Timer.delay(1);
            driveStraight(0);
            //charge(14);
            while(isAutonomous() && isEnabled())
            {
                if(vision.isHot || autoTime.get() > 5)
                {
                    solenoidShooter.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(1);
                    solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                    break;
                }
            }
            while(isAutonomous() && isEnabled())
            {
                windup();
                /*straightDriveTimer.start();
                 if(straightDriveTimer.get() < 1)
                 {
                 driveStraight(0.5);
                 }
                 else
                 {
                 setRightSpeed(0);
                 setLeftSpeed(0);
                 straightDriveTimer.stop();
                 }

                 straightDriveTimer.stop();*/
            }
        }
        else //TO DO
        {
            intake.set(0.4);
            Timer.delay(0.2);
            intake.set(0);
            driveStraight(0.4);
            Timer.delay(1);
            driveStraight(0);
            solenoidShooter.set(DoubleSolenoid.Value.kForward);
            Timer.delay(1);
            solenoidShooter.set(DoubleSolenoid.Value.kReverse);
            while(isAutonomous() && isEnabled())
            {

                windup();
                if(limCatapult.get())
                {
                    intake.set(0.5);
                    Timer.delay(0.5);
                    intake.set(0);
                    solenoidShooter.set(DoubleSolenoid.Value.kForward);
                    Timer.delay(1);
                    solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                    break;
                }
            }
            while(isAutonomous() && isEnabled())
            {
                windup();
                /*straightDriveTimer.start();
                if(straightDriveTimer.get() < 1)
                {
                    driveStraight(0.5);
                }
                else
                {
                    setRightSpeed(0);
                    setLeftSpeed(0);
                    straightDriveTimer.stop();
                }*/
                return;
            }
        }

    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl()
    {
        compressor.start();
        solenoidShooter.set(DoubleSolenoid.Value.kReverse); //Pulled in
        solenoidArm.set(DoubleSolenoid.Value.kForward); //In
        windupFlag = false;
        catapault1.set(0);
        catapault2.set(0);
        autoTime.reset();
         autoTime.stop();
        while(isOperatorControl() && isEnabled())
        {
            checkBattery();
            if(MorseCode.isDone)
            {
                (new Thread(new MorseCode("SOS", cameraLight))).start();
            }
            // Swapped the lower and raise intake buttons first period 2/18. - Bonnie
            if(xBox.getRawButton(5))
            {
                lowerIntake();
            }
            else if(xBox.getRawButton(6))
            {
                raiseIntake();
            }
            intake();

            if(throttle.getRawButton(2))
            {
                fire();
            }
            else if(shooterTimer.get() > 1)
            {
                solenoidShooter.set(DoubleSolenoid.Value.kReverse);
                shooterTimer.reset();
                shooterTimer.stop();
            }
            if(xBox.getRawButton(1))
            {
                windupFlag = true;
            }
            else if(xBox.getRawButton(2))
            {
                windupFlag = false;
                catapault1.set(0);
                catapault2.set(0);
            }
            if(windupFlag)
            {
                windup();
            }
            SmartDashboard.putDouble("Joystick: ", throttle.getRawAxis(2));
            SmartDashboard.putDouble("Wheel: ", swAdjust(wheel.getAxis(Joystick.AxisType.kX)));
            SmartDashboard.putDouble("Intake: ", xBox.getRawAxis(2));
            ultrasonicDistance();
            drive();

        }



        /**
         * This function is called once each time the robot enters test mode.
         */
    }

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
/*
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
