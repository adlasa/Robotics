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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotTemplate extends SimpleRobot
{
    public final int DIGITAL_MODULE_SLOT = 1;
    public final int ANALOG_MODULE_SLOT = 1;
    public final int SOLENOID_MODULE_SLOT = 1;
    public final double MAX_MOTOR_POWER_FOR_COMPRESSION = 3;
    public final double IDEAL_SHOOTING_DISTANCE = 14;
    //uses 1 & 2
    Gyro gyro = new Gyro(ANALOG_MODULE_SLOT, 1);
    Joystick steerWheel = new Joystick(1);
    Joystick throttle = new Joystick(2);
    Joystick xbox = new Joystick(3);
    Victor leftDrive1 = new Victor(1);
    Victor leftDrive2 = new Victor(2);
    Victor leftDrive3 = new Victor(3);
    Victor rightDrive1 = new Victor(4);
    Victor rightDrive2 = new Victor(5);
    Victor rightDrive3 = new Victor(6);
    Victor intake = new Victor(7);
    Victor catapult1 = new Victor(8);
    Victor catapult2 = new Victor(9);
    Relay intake1 = new Relay(1);
    Relay intake2 = new Relay(2);
    Relay catapultFire = new Relay(3);
    DriverStation stupidDriverStation = DriverStation.getInstance();
    AnalogChannel ultrasonic = new AnalogChannel(3);
    JoystickButton Button1 = new JoystickButton(steerWheel, 1);
    JoystickButton Button2 = new JoystickButton(steerWheel, 2);
    JoystickButton Button3 = new JoystickButton(throttle, 1);
    final int AREA_MINIMUM = 150;
    Compressor compressor1 = new Compressor(1, 1);
    DoubleSolenoid solenoidArm1 = new DoubleSolenoid(7, 1, 2);
    DoubleSolenoid solenoidArm2 = new DoubleSolenoid(7, 3, 4);
    DoubleSolenoid solenoidShooter = new DoubleSolenoid(7, 5, 6);
    ADXL345_I2C accel = new ADXL345_I2C(DIGITAL_MODULE_SLOT, ADXL345_I2C.DataFormat_Range.k2G);
    ADXL345_I2C.AllAxes axes = new ADXL345_I2C.AllAxes();
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

    public void compressorCheckThingy()
    {
        if((Math.abs(leftDrive1.get()) + Math.abs(leftDrive2.get()) + Math.abs(leftDrive3.get()) + Math.abs(rightDrive1.get()) + Math.abs(rightDrive2.get()) + Math.abs(rightDrive3.get())) < MAX_MOTOR_POWER_FOR_COMPRESSION)
        {
            compressor1.start();
        }
        else
        {
            compressor1.stop();
        }
    }

    public void superDrive(double power, double direction)
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
            cP = (distance - ultrasonicDistance)*kP;
            power = 1.0*cP;

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
            if (Math.abs(ultrasonicDistance-distance) < 0.25) {
                break;
            }
            bothSet(0);
        }
        
    }

    public void leftSet(double lDp)
    {
        leftDrive1.set(lDp);
        //leftDrive2.set(lDp);
        //leftDrive3.set(lDp);
        SmartDashboard.putDouble("Left Drive Power", lDp);
    }

    public void rightSet(double rDp)
    {
        rightDrive1.set(-rDp);
        //rightDrive2.set(-rDp);
        //rightDrive3.set(-rDp);
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

    public void lowerIntake()
    {
        // motor on intake
        solenoidArm1.set(DoubleSolenoid.Value.kForward);
        solenoidArm2.set(DoubleSolenoid.Value.kForward);
        waitBrendan(1);//change
        solenoidArm1.set(DoubleSolenoid.Value.kOff);
        solenoidArm2.set(DoubleSolenoid.Value.kOff);
        // stop motor on intake?
    }

    public void raiseIntake()
    {
        solenoidArm1.set(DoubleSolenoid.Value.kReverse);
        solenoidArm2.set(DoubleSolenoid.Value.kReverse);
        waitBrendan(1);//change 
        solenoidArm1.set(DoubleSolenoid.Value.kOff);
        solenoidArm2.set(DoubleSolenoid.Value.kOff);
    }

    public void expel() {
        //kick out the ball
        intake.set(-1);
        lowerIntake();
        intake.set(0);
        raiseIntake();
    }
    
    public void intake()
    {
        lowerIntake();
        intake.set(1);//could be reversed
        raiseIntake();
        intake.set(0);
    }

    public void robotInit()
    {
    }

    public void fire()
    {
        //do whatever the heck the shooter team made to make it shoot thingies at the other thingies
        catapultFire.setDirection(Relay.Direction.kBoth/*change to what make it shoot things*/);
        waitBrendan(1);//change to whatever it takes to fire
        catapultFire.setDirection(Relay.Direction.kBoth/*change to make it whatever it stops shooting the thingies */);
        solenoidShooter.set(DoubleSolenoid.Value.kReverse);
        waitBrendan(1);
        solenoidShooter.set(DoubleSolenoid.Value.kForward);
    }

    public void outputAccelData()
    {
        axes = accel.getAccelerations();
        SmartDashboard.putDouble("X Acceleration", axes.XAxis);
        SmartDashboard.putDouble("Y Acceleration", axes.YAxis);
        SmartDashboard.putDouble("Z Acceleration", axes.ZAxis);
    }

    //Adjust the steering whell input to normalize from -1 to 1
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

    public void waitBrendan(double time)
    {
        Timer wait = new Timer();
        //wait.start();
        wait.delay(time);
        /*
        while(wait.get() < time)
        {
            System.out.println("Party time!");
            //Party Time !!!
        }
        * */
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
        distance = ultrasonicDistance();
        turnSet(1);
        waitBrendan(0.1);
        bothSet(0);

        if(ultrasonicDistance() > distance)
        {
            turnSet(-0.4);
            while(isEnabled())
            {
                if(ultrasonicDistance() > distance)
                {
                    break;
                }
                distance = ultrasonicDistance();
            }
        }
        else
        {
            turnSet(0.4);
            while(isEnabled())
            {
                if(ultrasonicDistance() > distance)
                {
                    break;
                }
                distance = ultrasonicDistance();
            }
        }
        wallDistance(IDEAL_SHOOTING_DISTANCE);
        fire();
    }

    public void autonomous()
    {
        /*
         compressor1.start();
         solenoidShooter.set(DoubleSolenoid.Value.kForward);
         * */
        VisionThingy vision = new VisionThingy();
        final double IDEAL_DISTANCE = IDEAL_SHOOTING_DISTANCE;
        double[][] horizontalTargetLocations;
        double[][] verticalTargetLocations;
        if(stupidDriverStation.getDigitalIn(4))
        {
            System.out.println("Not doing anything");
            return;
        }
        /*
         if(stupidDriverStation.getDigitalIn(5)) {
         superDrive(0.4, 0);
         Timer heartbeat = new Timer();
         heartbeat.start();
         double ultrasonicDistance;
         while(isAutonomous() && isEnabled()) {
         compressorCheckThingy();
         ultrasonicDistance = ultrasonicDistance();
         SmartDashboard.putDouble("Heartbeat", heartbeat.get());
         SmartDashboard.putDouble("Ultrasonic Distance", ultrasonicDistance);
         heartbeat.reset();
         heartbeat.start();
         if(ultrasonicDistance >= IDEAL_DISTANCE && ultrasonicDistance <= IDEAL_DISTANCE+0.5)
         {
         superDrive(0,0);
         System.out.println("I'm here!");
         }
         else if(ultrasonicDistance > IDEAL_DISTANCE+0.5)
         {
         superDrive(0.3, 0);
         SmartDashboard.putString("Forward/Back", "Forward");
         }
         else if(ultrasonicDistance < IDEAL_DISTANCE)
         {
         superDrive(-0.3, 0);
         SmartDashboard.putString("Forward/Back", "Backward");
         }
         else
         {
         System.out.println("Something is going wrong I don't know what happening aaaaaaaaaahhhhhhhhhhhh");
         break;
         }
         }
         bothSet(0);
         System.out.println("Pew pew pew");
         }
         */

        //fire with pickup
        if(stupidDriverStation.getDigitalIn(2))
        {
            while(isAutonomous() && isEnabled())
            {
                compressorCheckThingy();
                horizontalTargetLocations = vision.horizontalTargetLocations();
                verticalTargetLocations = vision.verticalTargetLocations();
                Timer timer = new Timer();

                computerAssistedFire();
                intake();
                fire();
                checkBattery();
            }
        }
        //Fire without pickup
        if(stupidDriverStation.getDigitalIn(3))
        {
            vision.mainVision();
            if(vision.isHot())
            {
                while(isAutonomous() && isEnabled())
                {
                    compressorCheckThingy();
                    horizontalTargetLocations = vision.horizontalTargetLocations();
                    verticalTargetLocations = vision.verticalTargetLocations();
                    computerAssistedFire();
                    checkBattery();
                }
            }
            else
            {
                
                waitBrendan(5);
                while(isAutonomous() && isEnabled())
                {
                    compressorCheckThingy();
                    horizontalTargetLocations = vision.horizontalTargetLocations();
                    verticalTargetLocations = vision.verticalTargetLocations();
                    computerAssistedFire();
                    checkBattery();
                }
            }

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
                SmartDashboard.putDouble("Ultrasonic Distance", ultrasonicDistance());
                if(!stupidDriverStation.getDigitalIn(1))
                {
                    if(Button1.get() || Button2.get())
                    {
                        turnSet(swAdjust(steerWheel.getAxis(Joystick.AxisType.kX)));
                    }
                    else
                    {
                        superDrive(-throttle.getRawAxis(2), swAdjust(steerWheel.getAxis(Joystick.AxisType.kX)) * 180);
                    }
                }
                else
                {
                    leftSet(xbox.getRawAxis(2));
                    rightSet(xbox.getRawAxis(5));
                }
                
                SmartDashboard.putDouble("Gyro", gyro.getAngle());
                //intake
                if(throttle.getRawButton(4) || xbox.getRawButton(4)) // Y on xbox
                {
                    intake();
                }
                // firing using the trigger
                if(throttle.getRawButton(3) || xbox.getRawButton(2)) // B on xbox
                {
                    fire();
                }
                //firing with computer help
                if(throttle.getRawButton(2) || xbox.getRawButton(3)) // X on xbox
                {
                    computerAssistedFire();
                }
                if(throttle.getRawButton(5) || xbox.getRawButton(1)) {
                    expel();
                }
            }
        }
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test()
    {
        while(isTest() && isEnabled())
        {
            //outputAccelData();
            SmartDashboard.putDouble("Ultrasonic Distance", ultrasonicDistance());
            checkBattery();
        }
    }
}