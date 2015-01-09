/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Victor;
/**
 *
 * @author 1458
 */




public class InputBase
{
    //begin constants
    final int LEFTDRIVE1PORT = 1;
    final int LEFTDRIVE2PORT = 2;
    final int LEFTDRIVE3PORT = 3;
    final int RIGHTDRIVE1PORT = 4;
    final int RIGHTDRIVE2PORT = 5;
    final int RIGHTDRIVE3PORT = 6;
    //end constants
    
    //begin motors & etc declaring
    protected Victor leftDrive1 = new Victor(LEFTDRIVE1PORT);
    protected Victor leftDrive2 = new Victor(LEFTDRIVE2PORT);
    protected Victor leftDrive3 = new Victor(LEFTDRIVE3PORT);
    protected Victor rightDrive1 = new Victor(RIGHTDRIVE1PORT);
    protected Victor rightDrive2 = new Victor(RIGHTDRIVE2PORT);
    protected Victor rightDrive3 = new Victor(RIGHTDRIVE3PORT);
    //end motors & etc declaring
    
    //constructor
    public InputBase() {
        //add stuff here
    }
    
    
    //Left speed set for all motors, may need to be made negative depending on which side motors are on
    protected void setLeftSpeed(double speed) {
        leftDrive1.set(speed);
        leftDrive2.set(speed);
        leftDrive3.set(speed);
        return;
    }
        
    //Right speed set for all motors, already made negative, may need to be made positive
    protected void setRightSpeed(double speed) {
        rightDrive1.set(-speed);
        rightDrive2.set(-speed);
        rightDrive3.set(-speed);
        return;
    }
        
    //Left speed set for all motors
    protected void setLeftSpeedManual(double speed) {
        leftDrive1.set(speed);
        leftDrive2.set(speed);
        leftDrive3.set(speed);
        return;
    }
        
    //Right speed set for all motors
    protected void setRightSpeedManual(double speed) {
        rightDrive1.set(speed);
        rightDrive2.set(speed);
        rightDrive3.set(speed);
        return;
    }
    
    
}
