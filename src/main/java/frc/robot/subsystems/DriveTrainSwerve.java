/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.function.Supplier;

import edu.wpi.first.wpiutil.math.Matrix;

import com.analog.adis16470.frc.ADIS16470_IMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.wpiutil.math.Matrix;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
///////////////////////////////////////////////////////////////////////////////////////////////
public class DriveTrainSwerve extends SubsystemBase {
  /**
   * Creates a new DriveTrainSwerve.
   */
  //private final WPI_TalonFX m_leftFront;
  //private final WPI_TalonFX m_rightFront;
  //private final WPI_TalonFX m_leftBack;
  //private final WPI_TalonFX m_rightBack;
  private final WPI_VictorSPX m_steerMotorLF;
  private final WPI_TalonSRX m_driveMotorLF;
  private final WPI_VictorSPX m_steerMotorLB;
  private final WPI_TalonSRX m_driveMotorLB;
  private final WPI_VictorSPX m_steerMotorRF;
  private final WPI_TalonSRX m_driveMotorRF;
  private final WPI_VictorSPX m_steerMotorRB;
  private final WPI_TalonSRX m_driveMotorRB;
  
  private final AnalogInput m_steerEncoderLF;
  private final AnalogInput m_steerEncoderLB;
  private final AnalogInput m_steerEncoderRF;
  private final AnalogInput m_steerEncoderRB;

  private final double pi = Math.PI;
  private final double wheeldiam = 6.0; // in inches
  private final double wheelcirc = pi * wheeldiam / 12; //in feet
  private final double[] gearing = {1,1,1,1}; // probably like .1 or something small
  private final double wheelmaxvel = 15.0; // maximum velocity of a single wheel in feet per second. Needs to be empiracally tested, big tuning factor. UPDATE: might just bypass this and do power manipulation instead.
  private final double[][] wheelplacementsdefault = {{-11,11},{-11,-11},{11,11},{11,-11}};
  private final double[][] wheelplacementsalternate = {{-11,11},{-11,-11},{11,11},{11,-11}};
  private final double[][] unitcrossdefault = unitcrosscalc();
  private final double[] wheeldistancesdefault = wheeldistancescalc();
  public boolean defaultpointofrotation=true; // Note: Never set this to false the first time it is run unless you want bad default values
  private ADIS16470_IMU gyro;
  public double leftYAxis;
  public double leftXAxis;
  public double rightXAxis;
  
  
///////////////////////////////////////////////////////////////////////////////////////////////
  public DriveTrainSwerve(ADIS16470_IMU gyroname) {
  // Competition Robot motors
  //m_leftFront = new WPI_TalonFX( Constants.CANID_driveLF );
  //m_rightFront = new WPI_TalonFX( Constants.CANID_driveRF );
  //m_leftBack = new WPI_TalonFX( Constants.CANID_driveLB);
  //m_rightBack = new WPI_TalonFX( Constants.CANID_driveRB );
  //neutralCoast();
  gyro = gyroname;

  m_steerMotorLF = new WPI_VictorSPX(Constants.CANID_steerMotorLF);  
  m_driveMotorLF = new WPI_TalonSRX(Constants.CANID_driveMotorLF);
  m_steerMotorLB = new WPI_VictorSPX(Constants.CANID_steerMotorLB);
  m_driveMotorLB = new WPI_TalonSRX(Constants.CANID_driveMotorLB);
  m_steerMotorRF = new WPI_VictorSPX(Constants.CANID_steerMotorRF);
  m_driveMotorRF = new WPI_TalonSRX(Constants.CANID_driveMotorRF);
  m_steerMotorRB = new WPI_VictorSPX(Constants.CANID_steerMotorRB);
  m_driveMotorRB = new WPI_TalonSRX(Constants.CANID_driveMotorRB);

  m_steerEncoderLF = new AnalogInput(Constants.CANID_steerEncoderLF);
  m_steerEncoderLB = new AnalogInput(Constants.CANID_steerEncoderLB);
  m_steerEncoderRF = new AnalogInput(Constants.CANID_steerEncoderRF);
  m_steerEncoderRB = new AnalogInput(Constants.CANID_steerEncoderRB);


  }
///////////////////////////////////////////////////////////////////////////////////////////////
//public double euclid(double x, double y){return Math.sqrt(Math.pow(x,2)+Math.pow(y,2));} //this is just Math.hypot(x,y)

public double[] getdirections() // Currently unused
{
  double dirLF=0; //get these from hardware somehow
  double dirLB=0;
  double dirRF=0;
  double dirRB=0;
  double[] dirs = { dirLF, dirLB, dirRF, dirRB };
  return dirs;
}

public double[] getspeeds() // Currently unused
{
  double rpsLF=10; //get these from hardware somehow
  double rpsLB=10;
  double rpsRF=10;
  double rpsRB=10;
  double[] rots = { rpsLF, rpsLB, rpsRF, rpsRB };
  double[] vels = new double[rots.length];
  for(int i=0;i<rots.length;i++){vels[i] = wheelcirc*gearing[i]*rots[i];};
  return vels;
}

public double gyrozero() // Returns an ofset used to make the robots right side and zero degrees coincide
{
  return 0; //If we want, we can make this dynamically adjust on button press, for now this is a constant value
}

public double mod(double a,double b) // This is the mathematical definition of modulus. Do not touch this.
{
  return ((a%b)+b)%b;
}

public double gyroang() //get gyro angle in radians counterclockwise from the gyro's internal zero.
{
  return mod(-1*this.gyro.getAngle()/360*pi*2,pi*2);
}

public double gyrodir()// get angle in radians counterclockwise from our set gyro zero 
{
  double gyrodir = mod(this.gyroang()+this.gyrozero(),2*pi); 
  return gyrodir; 
}

public double[][] rotmatrix()
{
  double gyrodirconst = this.gyrodir();
  double[][] rotmatrix = {{Math.cos(gyrodirconst),-Math.sin(gyrodirconst)},{Math.sin(gyrodirconst),Math.cos(gyrodirconst)}};
  return rotmatrix;
}

public double safety()
{
  boolean safetyon = false;// get safety mode toggle status
  double safetyoffval = Constants.safetyoff; 
  double safetyonval = Constants.safetyon;
  double safety;
  if(safetyon){safety = safetyonval;}
  else {safety = safetyoffval;}
  return safety;
}

public double[] joy2vecxy()
{
  double x = this.leftXAxis;//get joystick x value
  double y = this.leftYAxis;//get joystick y value
  double deadbandxy = Constants.deadbandxy;
  if(Math.hypot(x, y)<deadbandxy){return new double[] {0,0};}
  else {return new double[] {this.safety()*x,this.safety()*y};}
}

public double joy2vecz()
{
  double z = this.rightXAxis;//get joystick z value
  double deadbandz = Constants.deadbandz;
  if(Math.abs(z)<deadbandz) {return 0.0;}
  else {return Math.copySign(Math.pow(Math.abs(this.safety()*z),1.5),-1*z);}
  // this takes an arbitrary power of the magnitude of z, and assigns it negative the sign of z such that pushing left on the joystick coresponds to rotating counterclockwise, the direction of positive angle.
  // powers of z^x such that x>=1 have z=0 map to 0 and z=1 map to one, where z is small the function grows slowly and where z is close to 1 it grows faster, so optimizes for precision for small angles.
}

public double[][] unitcrosscalc() //creates a set of counterclockwise perpendicular vectors scaled down to unit length for rotation component of motion using default wheelplacements
{
  double[][] unitcrossholder = this.wheelplacements();
  for(int i=0;i<unitcrossholder.length;i++)
  {
    double length = Math.hypot(-1*unitcrossholder[i][1],unitcrossholder[i][0]);
    unitcrossholder[i] = new double[] {-1*this.wheelplacementsdefault[i][1]/length,this.wheelplacementsdefault[i][0]/length};
  }
  return unitcrossholder;
}

public double[][] wheelplacements() //Returns the wheel placement settup that is currently active
{
  if(this.defaultpointofrotation){return this.wheelplacementsdefault;}
  else{return this.wheelplacementsalternate;}
}

public double[][] unitcross() //Returns the unit cross vectors for the setup that is currently active
{
  if(this.defaultpointofrotation){return this.unitcrossdefault;}
  else{return this.unitcrosscalc();} 
}

public double[] wheeldistancescalc() //Returns how fare each wheel is from the center of rotation
{
  double[] holder = new double[this.wheelplacements().length];
  for(int i=0;i<this.wheelplacements().length;i++)
  {
    holder[i] = Math.hypot(this.wheelplacements()[i][0], this.wheelplacements()[i][1]);
  }
  return holder;
}

public double[] wheeldistances() //Returns a list of the wheel distances from the point of rotation currently active
{
  if(this.defaultpointofrotation){return this.wheeldistancesdefault;}
  else{return this.wheeldistancescalc();}
}

public double[] wheelsfromcenter() //Returns how far each wheel is from the center as a proportion of how far the furthest is
{
  double max = this.wheeldistances()[0];
  for(int i=1;i<this.wheeldistances().length;i++)
  {
    if(this.wheeldistances()[i]>max){max = this.wheeldistances()[i];}
  }
  double[] holder = this.wheeldistances();
  for(int i = 0;i<this.wheeldistances().length;i++)
  {
    holder[i] = this.wheeldistances()[i]/max;
  }
  return holder;
}

public double xylength() //Returns the magnitude of the xy joystick
{
  double[] holder = this.joy2vecxy();
  return Math.hypot(holder[0], holder[1]);
}

public double normalize() //Returns the total magnitude of all inputs
{
  return Math.abs(this.joy2vecz())+this.xylength();
}

public double maxduetospin() //Returns a signed proportion of total input magnitude that is used for rotation
{
  return this.joy2vecz()/this.normalize();
}

public double maxduetovel() //Returns a proportion of total input magnitude that is used for regular movement
{
  return this.xylength()/this.normalize();
}

public double[] times(double[] pair,double solo) //Quick tool for scalar multiplication of vectors
{
  return new double[] {pair[0]*solo,pair[1]*solo};
}

public double[][] rottovectors() //Returns a set of vectors indicating where the rotation component of movement would vector each wheel to without regard for the gyro
{
  double maxduetospin = this.maxduetospin();
  double[][] unitcross = this.unitcross();
  double[] wheelsfromcenter = this.wheelsfromcenter();
  double[][] holder = this.wheelplacements();
  for(int i=0;i<unitcross.length;i++)
  {
    holder[i] = this.times(unitcross[i],wheelsfromcenter[i]*maxduetospin);
  }
  return holder;
}

public double[][] added() //Returns a set of vectors indicating the combined rotation and linear motion that each wheel should be vectored to without regard for the gyro
{
  double maxduetovel = this.maxduetovel();
  double[] joy2vecxy = this.joy2vecxy();
  double[][] rottovectors = this.rottovectors();
  double[][] holder = rottovectors;
  for(int i=0;i<rottovectors.length;i++)
  {
    holder[i] = new double[] {rottovectors[i][0]+maxduetovel*joy2vecxy[0],rottovectors[i][1]+maxduetovel*joy2vecxy[1]};
  }
  return holder;
}

public double[][] wheelheadings() //Returns a set of vectors that each wheel should be vecotred to with regard to the gyro.
{
  double[][] rotmatrix = this.rotmatrix();
  double[][] added = this.added();
  // alright, I am gonna hardcode this section, but what I am hardcoding is just matrix transpose operations and dot product of matricies Transpose(Dot([rotmatrix],Transpose([added])))
  // I couldn't find a library that I was happy with using.
  // Just think of it as a coordinate system adjustment(rotation in this case) so each vector gets rotated but we only have to do addition and multiplication (in a very specific way) to get the result.
  double a = rotmatrix[0][0];
  double b = rotmatrix[0][1];
  double c = rotmatrix[1][0];
  double d = rotmatrix[1][1];
  double e = added[0][0];
  double f = added[0][1];
  double g = added[1][0];
  double h = added[1][1];
  double i = added[2][0];
  double j = added[2][1]; //lol skipped k, not gonna redo it though.
  double l = added[3][0];
  double m = added[3][1];
  double[][] holder = {{a*e+b*f,c*e+d*f},{a*g+b*h,c*g+d*h},{a*i+j*j,c*i+d*j},{a*l+b*m,c*l+d*m}};
  return holder;
}

public void drivebyjoystick() //The main function to be called. Sets each individual swerve wheel setup to do its thing.
{
  double[][] wheelheadings = this.wheelheadings();
  swervemodule(wheelheadings[0], m_driveMotorLF, m_steerMotorLF, m_steerEncoderLF);
  swervemodule(wheelheadings[1], m_driveMotorLB, m_steerMotorLB, m_steerEncoderLB);
  swervemodule(wheelheadings[2], m_driveMotorRF, m_steerMotorRF, m_steerEncoderRF);
  swervemodule(wheelheadings[3], m_driveMotorRB, m_steerMotorRB, m_steerEncoderRB);
  return;
}

public void swervemodule(double[] input, WPI_TalonSRX drive, SpeedController steer, AnalogInput encoder) //Does angle math and sets steering and drive motors
{
  double volt = encoder.getVoltage();
  double currentangle = volt/5*pi*2; 
  // Encoders should read 0 when all wheels are facing perfectly right, and should be PI/2 when perfectly foreward. 
  // If not, we need to make adjustments so their offset from that is taken care of.
  double magnitude = Math.hypot(input[0], input[1]);
  double dir = Math.atan2(input[1], input[0]);
  if (dir<0.0){dir=pi*2 + dir;}
// a mod b in its true math form is ((a % b) + b) % b in java.
  double a = dir-currentangle;
  double b = pi*2;
  double amodb = ((a % b) + b) % b;
  if(amodb<=pi/2 || amodb>=pi*3/2 ){steer.set(a/(pi/2));drive.set(magnitude);}
  if(amodb<pi*3/2 && amodb>pi/2){steer.set((a-pi)/(pi/2));drive.set(-magnitude);}
  // If the current angle is less than or equal to 90degrees from the desired angle, steer toward the desired angle proportionally to how far off you are, and set the drive power to be foreward.
  // If the current angle is between 90 and 180degrees from the desired angle, treat it as if you were the antipode of the current angle and steer toward that proportionally to how far from there you are, and set the drive power to be backward.
  return;
}

// public void neutralBrake() {
//   m_leftFront.setNeutralMode( NeutralMode.Brake);
//   m_rightFront.setNeutralMode( NeutralMode.Brake);
//   m_leftBack.setNeutralMode( NeutralMode.Brake);
//   m_rightBack.setNeutralMode( NeutralMode.Brake);
// }

// public void neutralCoast() {
//   m_leftFront.setNeutralMode( NeutralMode.Coast);
//   m_rightFront.setNeutralMode( NeutralMode.Coast);
//   m_leftBack.setNeutralMode( NeutralMode.Coast);
//   m_rightBack.setNeutralMode( NeutralMode.Coast);
// }
//////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
