/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author 1458
 */
public class InputWheel extends InputBase
{
    //begin constant
    final int JOYSTICKPORT = 2;
    final int WHEELPORT = 1;
    //end constants
    
    //constructor
    public InputWheel() {
        //add constructor stuff
    }
    
    //variables
    double pCorrection;
    double iCorrection;
    double totalCorrection;
    final double kP = 0.25, kI = 1.0;
    double direction;
    double power;
    //end variables
    
    public void drive() {

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
    
}
