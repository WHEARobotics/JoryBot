/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc3881;
//Test code for the Traction Device Pneumatic Section
//Test code for the Traction Drive Drive Section
//Code to Scale the wheel speed for main drive
//Test code for the Shooter Calculations
//Test code for the Shooter
//Test code for the Bridge Arm
//Test code for the Harvester
//Test code for the Autonomous
//Third joystick for shooting added
//Default mode added: if Z axis is at bottom, then the targeting is replaced by 3 fixed speeds
//ToDo's have been completed
//Look at Shooter Syntax, and remove extranious bits. 


import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class ASimpleJavaBot extends SimpleRobot {
	//Constants
	static final double GRAVITY = 9.8;
	static final double TANTHETA = Math.tan(Math.toRadians(46));
	static final double COSTHETA = Math.cos(Math.toRadians(46));
	static final double RANGECOMP = 0.1524;
	static final double THROTTLESCALAR = 3;
	static final double SECONDARYSCALAR = 5;//Secondary drive enabled, main drive needs to be slowed
	static final double TOPBASKET = 2.49-0.6604;
	static final double CENTRALBASKETS = 1.55-0.6604;
	static final double SPINUP = 1;
	static final double SHOOT = 11;
	static final double END = 15;
	//Object Creation
	RobotDrive drive = new RobotDrive(2,4,1,3);
	Joystick rightStick = new Joystick(1);
	Joystick leftStick = new Joystick(2);
	Joystick shooterStick = new Joystick(3);
	Solenoid tractionDeploy = new Solenoid(1);
	Solenoid tractionRetract = new Solenoid(2);
	Compressor compressor = new Compressor(3,1);
	Victor tractionDriveRight = new Victor(7);
	Victor tractionDriveLeft = new Victor(8);
	Relay Collector = new Relay(3);
	Victor bridgeArm = new Victor(10);
	Jaguar shooterJaguarRight = new Jaguar(6);
	Jaguar shooterJaguarLeft = new Jaguar(5);
	Ultrasonic rangeFinder = new Ultrasonic(1,2, Ultrasonic.Unit.kMillimeter);
	Timer timer;
	public ASimpleJavaBot(){
		compressor.start();
		drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
		drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
	}
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
    	while(isAutonomous()){
    		double jaguarSpeedAuto = 0.71;
    		Collector.setDirection(Relay.Direction.kReverse);
    		Collector.set(Relay.Value.kOn);
    		shooterJaguarRight.set(jaguarSpeedAuto);
    		shooterJaguarLeft.set(-jaguarSpeedAuto);
    		SmartDashboard.putDouble("Jaguar Speed", jaguarSpeedAuto);	
    	}
    }

    /**
     * This function is called once each time the robot enters operator control.

     */
    public void operatorControl() {
    	double rangeMM = 0;
    	boolean Trigger = false;
    	double jaguarSpeedAuto = 0;
		shooterJaguarRight.set(jaguarSpeedAuto);
		shooterJaguarLeft.set(jaguarSpeedAuto);
		SmartDashboard.putDouble("Jaguar Speed", jaguarSpeedAuto);
		while(isOperatorControl()){
        	Trigger = shooterStick.getRawButton(1);
    		boolean ballCollectorForward = rightStick.getRawButton(3);
        	boolean ballCollectorBackward = rightStick.getRawButton(2);
    		double override = shooterStick.getAxis(Joystick.AxisType.kZ);
    		double xInput;
    		double yInput;
    		double rotation;
    		double xInputRounded = rightStick.getAxis(Joystick.AxisType.kX);
    		double yInputRounded = rightStick.getAxis(Joystick.AxisType.kY);
    		double rotationRounded = leftStick.getAxis(Joystick.AxisType.kY);
    		boolean deploy = leftStick.getRawButton(2);
    		if(deploy == true){
    			tractionRetract.set(false);
    			tractionDeploy.set(true);
    			tractionDriveRight.set(rightStick.getAxis(Joystick.AxisType.kY)*-1);
    			tractionDriveLeft.set(rightStick.getAxis(Joystick.AxisType.kY)*-1);
    			xInput = 0;
    			yInput = rightStick.getAxis(Joystick.AxisType.kY)/3;
    			rotation = leftStick.getAxis(Joystick.AxisType.kX)/3;
    			}
    		else{
    			tractionDeploy.set(false);
    			tractionRetract.set(true);
    			tractionDriveRight.set(0);
    			tractionDriveLeft.set(0);
    			xInput = rightStick.getAxis(Joystick.AxisType.kX);
    			yInput = rightStick.getAxis(Joystick.AxisType.kY);
    			rotation = leftStick.getAxis(Joystick.AxisType.kY);
    		}
    		//Harvester Code
    		if(ballCollectorForward == true){
        		if(ballCollectorBackward == true){
        			Collector.set(Relay.Value.kOff);
        			SmartDashboard.putString("Harvester: ","Off");
        		}
        		else{
        			Collector.setDirection(Relay.Direction.kReverse);
        			Collector.set(Relay.Value.kOn);
        			SmartDashboard.putString("Harvester: ","Backward");
        		}
        	}
        	else{
        		if(ballCollectorBackward == true){
        			Collector.setDirection(Relay.Direction.kForward);
        			Collector.set(Relay.Value.kOn);
        			SmartDashboard.putString("Harvester: ","Forward");
        		}
        		else{
        			Collector.set(Relay.Value.kOff);
        			SmartDashboard.putString("Harvester: ","Off");
        		}
        	}
    		//Shooter Code
    		boolean targetOne = shooterStick.getRawButton(4);
        	boolean targetTwo = shooterStick.getRawButton(5);
        	boolean keyShot = shooterStick.getRawButton(2);
        	boolean userControl = shooterStick.getRawButton(3);
        	boolean fullSpeed = shooterStick.getRawButton(7);
        			
        	//Decide which target to shoot at
 
        	if (override > .5) {

        		if((targetOne == true) ^ (targetTwo == true)){
        			rangeFinder.ping();
        			boolean rangeValid = rangeFinder.isRangeValid();
        			if(rangeValid == true){
        				rangeMM = rangeFinder.getRangeMM();
        				double jaguarSpeed;
        				if(targetOne == true){
        					jaguarSpeed = calcShooterSpeed(rangeMM,TOPBASKET);
        					SmartDashboard.putString("Target:","Top Basket");
        				}
        				else{
        					jaguarSpeed = calcShooterSpeed(rangeMM,CENTRALBASKETS);
        					SmartDashboard.putString("Target:", "Central Basket");
        				}
        				if(Trigger == true){
        					shooterJaguarRight.set(jaguarSpeed);
        					shooterJaguarLeft.set(-jaguarSpeed);
        					SmartDashboard.putString("Shooter:","Shooting");
                			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        				}
        				else{
        					shooterJaguarRight.set(0);
        					shooterJaguarLeft.set(0);
        					SmartDashboard.putString("Shooter:", "Not Shooting");
                			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        				}
        			}
        			else{
        				shooterJaguarRight.set(0);
        				shooterJaguarLeft.set(0);
        			}
        		}
        		else {
        			shooterJaguarRight.set(0);
        			shooterJaguarLeft.set(0);
        		}

        	}

        	else{


        		double jaguarSpeed;

        		if (targetOne) {
        			jaguarSpeed = 0.47;
        			SmartDashboard.putString("Target:", "Central Basket, 8 feet");
        			if(Trigger == true){
        				shooterJaguarRight.set(jaguarSpeed);
        				shooterJaguarLeft.set(-jaguarSpeed);
        				SmartDashboard.putString("Shooter:","Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}
        			else{
        				shooterJaguarRight.set(0);
        				shooterJaguarLeft.set(0);
        				SmartDashboard.putString("Shooter:", "Not Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}	
        		}
        		else if (targetTwo){
        			jaguarSpeed = 0.61;
        			SmartDashboard.putString("Target:", "Central Basket, 15 feet");
        			if(Trigger == true){
        				shooterJaguarRight.set(jaguarSpeed);
        				shooterJaguarLeft.set(-jaguarSpeed);
        				SmartDashboard.putString("Shooter:","Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}
        			else{
        				shooterJaguarRight.set(0);
        				shooterJaguarLeft.set(0);
        				SmartDashboard.putString("Shooter:", "Not Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}
        		}
        		else if (keyShot){
        			jaguarSpeed = 0.5;
        			SmartDashboard.putString("Target:", "Central Basket, 12 feet");
        			if(Trigger == true){
        				shooterJaguarRight.set(jaguarSpeed);
        				shooterJaguarLeft.set(-jaguarSpeed);
        				SmartDashboard.putString("Shooter:","Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}
        			else{
        				shooterJaguarRight.set(0);
        				shooterJaguarLeft.set(0);
        				SmartDashboard.putString("Shooter:", "Not Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}
        		}
        		if (userControl){
        			jaguarSpeed = (shooterStick.getAxis(Joystick.AxisType.kY)+1)/2;
        			SmartDashboard.putString("Target:", "Central Basket, 12 feet");
        			if(Trigger == true){
        				if(jaguarSpeed <0){
        					jaguarSpeed = 0;
        				}
        				shooterJaguarRight.set(jaguarSpeed);
        				shooterJaguarLeft.set(-jaguarSpeed);
        				SmartDashboard.putString("Shooter:","Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
            			SmartDashboard.putString("Target:", "User Control");
        			}
        			else{
        				shooterJaguarRight.set(0);
        				shooterJaguarLeft.set(0);
        				SmartDashboard.putString("Shooter:", "Not Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
            			SmartDashboard.putString("Target:", "User Control");
        			}
        	}
        		else if (fullSpeed){
        			jaguarSpeed = 1;
        			SmartDashboard.putString("Target:", "Central Basket, 12 feet");
        			if(Trigger == true){
        				shooterJaguarRight.set(jaguarSpeed);
        				shooterJaguarLeft.set(-jaguarSpeed);
        				SmartDashboard.putString("Shooter:","Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}
        			else{
        				shooterJaguarRight.set(0);
        				shooterJaguarLeft.set(0);
        				SmartDashboard.putString("Shooter:", "Not Shooting");
            			SmartDashboard.putDouble("Jaguar Speed", jaguarSpeed);
        			}
        		}
        	}
			
			
			
        	//Bridge Arm Code
        	boolean bridgeArmActivate = leftStick.getRawButton(1);
    		if(bridgeArmActivate== true){
    			bridgeArm.set(leftStick.getAxis(Joystick.AxisType.kY));
    		}
    		else{
    			bridgeArm.set(0);
    		}
    		xInputRounded = dashboardRounding(xInputRounded);
    		yInputRounded = dashboardRounding(yInputRounded);
    		rotationRounded = dashboardRounding(rotationRounded);
    		SmartDashboard.putDouble("Strafe", xInputRounded);
    		SmartDashboard.putDouble("Forward Backward", yInputRounded);
    		SmartDashboard.putDouble("Rotation", rotationRounded);
    		SmartDashboard.putDouble("Range", rangeMM);
    		SmartDashboard.putBoolean("Shooter", Trigger);
        	//Drive Train Code
    		drive.mecanumDrive_Cartesian(0,yInput,rotation,0);
    	}
    	

    }
    public double calcShooterSpeed(double range, double height){
    	//Range input in Millimeters
    	//Height of basket above shooter
    	//Returns value between 0 and 1 representing shooter jaguar speed
    	double rangeMeters = range/1000;
		rangeMeters = rangeMeters-RANGECOMP;
		double ViLinear;
		double ViRotational;
		double jaguarSpeed;
		ViLinear = rangeMeters*TANTHETA;
		ViLinear = ViLinear - height;
		ViLinear = 2*ViLinear;
		ViLinear = GRAVITY/ViLinear;
		ViLinear = Math.sqrt(ViLinear);
		double ViLinearptTwo = rangeMeters/COSTHETA;
		ViLinear = ViLinear*ViLinearptTwo;
		ViRotational = ViLinear/0.471;//0.471 is the circumfrence of the Shooter Wheel in Meters-
		jaguarSpeed = ViRotational/45.23;//Conversion between rotational velocity and jaguar speed input
		if(0<=jaguarSpeed && jaguarSpeed<= 1){
			return jaguarSpeed;
		}
		else{
			System.out.println("Not Valid Jaguar Input");
			return 1;
		}
    }
    public double dashboardRounding(double inputValue){
    	inputValue = inputValue*100;
    	inputValue = MathUtils.round(inputValue);
    	inputValue = inputValue/100;
    	return inputValue;
    }
    }
