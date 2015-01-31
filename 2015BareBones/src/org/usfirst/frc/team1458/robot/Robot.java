
package org.usfirst.frc.team1458.robot;


import org.usfirst.frc.team1458.robot.Levels.CarryObject;
import org.usfirst.frc.team1458.robot.Levels.LevelMod;
import org.usfirst.frc.team1458.robot.Levels.LevelMode;
import org.usfirst.frc.team1458.robot.Levels.MainLevel;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Gyro;


public class Robot extends SampleRobot {
	Joystick right = new Joystick(0);
	Joystick left = new Joystick(1);
	Joystick buttonPanel = new Joystick(2);	
	
	Encoder leftEncoder = new Encoder(0,1);//need proper channels
	Encoder rightEncoder = new Encoder(2,3);//need proper channels
	
	I2CGyro gyro = new I2CGyro();
	
	Infrared elevatorBottom = new Infrared(0,1);
	Infrared elevatorTop = new Infrared(1,1);
	Infrared toteCheck = new Infrared(2,2);
	Infrared seeRight = new Infrared(3,1);
	Infrared seeLeft = new Infrared(4,1);

	LED ledStrip = new LED();
	
	Levels levelHandler = new Levels();
	
	PowerDistributionPanel pdp = new PowerDistributionPanel();
    
    public Robot() {
    	//initialise all of the things, for the elevator, robotfunctions, gyro, etc.
    	RobotFunctions robot = new RobotFunctions();
    	Elevator elevator = new Elevator();
    
    }

    /**
     * Autonomous mode
     */
    public void autonomous() {
    
    }

    /**
     * operator controlled mode
     */
    public void operatorControl() {
    	
    	gyro.reset();
    	gyro.update();
    	double gyroAngle;
    	double gyroRate;
    	
    	double elevatorHeight;
    	
    	Levels.MainLevel mainLevel = Levels.MainLevel.ONE;
    	Levels.CarryObject carryObject = Levels.CarryObject.TOTE;
    	Levels.LevelMode levelMode = Levels.LevelMode.FLOOR;
    	Levels.LevelMod levelMod = Levels.LevelMod.LIP;
    	
    	
    	
    	rightEncoder.setDistancePerPulse(1.0);
    	leftEncoder.setDistancePerPulse(1.0);
    	
    	while(isEnabled()) {
    		gyroAngle = gyro.getAngle();
    		gyroRate = gyro.getRate();
    		System.out.println("Gyro Angle: " + gyroAngle);
    		System.out.println("GyroRate: " + gyroRate);
    		SmartDashboard.putNumber("Gyro Angle: ", gyroAngle);
    		SmartDashboard.putNumber("Gyro Rate: ", gyroRate);
    		SmartDashboard.putNumber("Gyro Angle Graph: ", gyroAngle);
    		SmartDashboard.putNumber("Gyro Rate Graph: ", gyroRate);
    		gyro.update();
    		
    		leftDriveFront.set(-left.getAxis(Joystick.AxisType.kY));
    		leftDriveRear.set(-left.getAxis(Joystick.AxisType.kY));
    		rightDriveFront.set(right.getAxis(Joystick.AxisType.kY));
    		rightDriveRear.set(right.getAxis(Joystick.AxisType.kY));
    		
    		if(buttonPanel.getRawButton(1)) {
    			mainLevel = Levels.MainLevel.ONE;
    		} else if (buttonPanel.getRawButton(2)) {
    			mainLevel = Levels.MainLevel.TWO;
    		}
    		
    		if(buttonPanel.getRawButton(5)) {
    			levelMode = Levels.LevelMode.FLOOR;
    			
    		} else if (buttonPanel.getRawButton(6)) {
    			levelMode = Levels.LevelMode.STEP;
    		}
    		
    		if(buttonPanel.getRawButton(8)) {
    			carryObject = Levels.CarryObject.TOTE;
    		} else if (buttonPanel.getRawButton(9)) {
    			carryObject = Levels.CarryObject.CONTAINER;
    		}
    		switch(levelMode) {
    		case FLOOR: {
    			buttonPanel.setOutputs(0);
    			buttonPanel.setOutput(1,true);
    		}
    		case STEP: {
    			buttonPanel.setOutputs(0);
    			buttonPanel.setOutput(2,true);
    		}
    		
    		}
    		
    		elevatorHeight=levelHandler.getHeight(mainLevel, levelMode, carryObject, levelMod);
    		SmartDashboard.putNumber("elevatorHeight", elevatorHeight);
    		
    		
    	}
    }
    
    

    /**
     * Runs during test mode
     */
    public void test() {
    	
    }
}
/*
double lDp=0;//left Drive power
double rDp=0;//right Drive power
double cDp=0;//centre Drive power

final double kPGyro=0.0;
final double kIGyro=0.0;
final double kDGyro=0.0;

final double kPEncoder=0.0;
final double kIEncoder=0.0;
final double kDEncoder=0.0;

double adjustPGyro;
double adjustIGyro;
double adjustDGyro;

double adjustPEncoder;
double adjustIEncoder;
double adjustDEncoder;

double adjustGyro;
double adjustEncoder;
double adjustTotal;

double idealGyroAngle = 0;

double power;
*/

/*
power=right.getY();

adjustPGyro=kPGyro*(gyro.getAngle()-idealGyroAngle);
adjustIGyro=kIGyro*0;//not applicable but included for consistency
adjustDGyro=kDGyro*gyro.getRate();//based on rotation to right in degrees/second
adjustGyro=adjustPGyro+adjustIGyro+adjustPGyro;

adjustPEncoder=kPEncoder*(rightEncoder.get()-leftEncoder.get());
adjustIEncoder=kIEncoder*0;//also not applicable
adjustDEncoder=kDEncoder*(leftEncoder.getRate()-rightEncoder.getRate());
adjustEncoder=adjustPEncoder+adjustIEncoder+adjustPEncoder;

adjustTotal=adjustGyro+adjustEncoder;
rDp=power-adjustTotal;
lDp=power+adjustTotal;


rightDriveFront.set(rDp);
rightDriveRear.set(rDp);
leftDriveFront.set(lDp);
leftDriveRear.set(lDp);
centreDrive.set(cDp);

System.out.println("Right: " + rDp);
SmartDashboard.putNumber("Right: ", rDp);

System.out.println("Left: " + lDp);
SmartDashboard.putNumber("Left: ", lDp);

System.out.println("Centre: "+cDp);
SmartDashboard.putNumber("Centre:",cDp);


System.out.println("Right encoder: "+rightEncoder.getRaw());
SmartDashboard.putNumber("Right encoder: ",rightEncoder.getRaw());

System.out.println("Left encoder: "+leftEncoder.getRaw());
SmartDashboard.putNumber("Left encoder: ",leftEncoder.getRaw());
*/