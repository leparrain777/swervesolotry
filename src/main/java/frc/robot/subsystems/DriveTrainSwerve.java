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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.ejml.simple.SimpleMatrix;
import edu.wpi.first.wpiutil.math.Matrix;

import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final double wheeldiam = 6.0; // in inches
  private final double wheelcirc = Math.PI * wheeldiam / 12; //in feet
  private final double[] gearing = {1,1,1,1}; // probably like .1 or something small
  private final double wheelmaxvel = 15.0; // maximum velocity of a single wheel in feet per second. Needs to be empiracally tested, big tuning factor.
  private final double[][] wheelplacementsdefault = {{-11,11},{-11,-11},{11,11},{11,-11}};
  private final double[][] unitcrossdefault = unitcrossdefault();
  private final double[] wheeldistancesdefault = wheeldistancesdefault();
  public boolean defaultpointofrotation=true;
  
///////////////////////////////////////////////////////////////////////////////////////////////
  public DriveTrainSwerve() {
  // Competition Robot motors
  //m_leftFront = new WPI_TalonFX( Constants.CANID_driveLF );
  //m_rightFront = new WPI_TalonFX( Constants.CANID_driveRF );
  //m_leftBack = new WPI_TalonFX( Constants.CANID_driveLB);
  //m_rightBack = new WPI_TalonFX( Constants.CANID_driveRB );
  //neutralCoast();
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

public double[] getdirections()
{
  double dirLF=0; //get these from hardware somehow
  double dirLB=0;
  double dirRF=0;
  double dirRB=0;
  double[] dirs = { dirLF, dirLB, dirRF, dirRB };
  return dirs;
}

public double[] getspeeds()
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

public double gyrozero()
{
  return Math.PI*0;//get gyro zero angle, this is our adjustment term.
}

public double gyroang()
{
  return Math.PI/2;//get gyro angle in radians counterclockwise from
}

public double gyrodir()
{
  double gyrodir = (this.gyroang()+this.gyrozero()+2*Math.PI)%(Math.PI*2); // get angle in radians counterclockwise from some set gyro zero 
  return gyrodir; 
}

public double[][] rotmatrix()
{
  //no longer needed
  double gyrodirconst = this.gyrodir();
  double[][] rotmatrix = {{Math.cos(gyrodirconst),-Math.sin(gyrodirconst)},{Math.sin(gyrodirconst),Math.cos(gyrodirconst)}};
  return rotmatrix;
}

public double safety()
{
  boolean safetyon = false;// get safety mode
  double safetyoffval = Constants.safetyoff; 
  double safetyonval = Constants.safetyon;
  double safety;
  if(safetyon){safety = safetyonval;}
  else {safety = safetyoffval;}
  return safety;
}

public double[] joy2vecxy()
{
  double x = .5;//get joystick x value
  double y = .5;//get joystick y value
  double deadbandxy = Constants.deadbandxy;
  if(Math.hypot(x, y)<deadbandxy){return new double[] {0,0};}
  else {return new double[] {this.safety()*x,this.safety()*y};}
}

public double joy2vecz()
{
  double z = .5;//get joystick z value
  double deadbandz = Constants.deadbandz;
  if(z<deadbandz) {return 0.0;}
  else {return Math.copySign(Math.pow(Math.abs(this.safety()*z),1.5),z);}
}

public double[][] unitcrossdefault()
{
  double[][] unitcrossholder = wheelplacementsdefault;
  for(int i=0;i<this.wheelplacementsdefault.length;i++)
  {
    double length = Math.hypot(this.wheelplacementsdefault[i][1],this.wheelplacementsdefault[i][0]);
    unitcrossholder[i] = new double[] {this.wheelplacementsdefault[i][1]/length,this.wheelplacementsdefault[i][0]/length};
  }
  return unitcrossholder;
}

public double[][] wheelplacements()
{
  if(this.defaultpointofrotation){return this.wheelplacementsdefault;}
  else{return this.wheelplacementsdefault;} // Here is where you make a different point of rotation work part 1
}

public double[][] unitcross()
{
  if(this.defaultpointofrotation){return this.unitcrossdefault;}
  else{return this.unitcrossdefault;} // Here is where you make a different point of rotation work part 2
}

public double[] wheeldistancesdefault()
{
  double[] holder = new double[this.wheelplacements().length];
  for(int i=0;i<this.wheelplacements().length;i++)
  {
    holder[i] = Math.hypot(this.wheelplacements()[i][0], this.wheelplacements()[i][1]);
  }
  return holder;
}

public double[] wheeldistances()
{
  if(this.defaultpointofrotation){return this.wheeldistancesdefault;}
  else{return this.wheeldistancesdefault;} // Here is where you make a different point of rotation work part 3
}

public double[] wheelsfromcenter()
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

public double xylength()
{
  double[] holder = this.joy2vecxy();
  return Math.hypot(holder[0], holder[1]);
}

public double normalize()
{
  return this.joy2vecz()+this.xylength();
}

public double maxduetospin()
{
  return this.joy2vecz()/this.normalize();
}

public double maxduetovel()
{
  return this.xylength()/this.normalize();
}

public double[] times(double[] pair,double solo)
{
  return new double[] {pair[0]*solo,pair[1]*solo};
}

public double[][] rottovectors()
{
  double maxduetospin = this.maxduetospin();
  double[][] unitcross = unitcross();
  double[] wheelsfromcenter = this.wheelsfromcenter();
  double[][] holder = wheelplacements();
  for(int i=0;i<unitcross.length;i++)
  {
    holder[i] = this.times(unitcross[i],wheelsfromcenter[i]*maxduetospin);
  }
  return holder;
}

public double[][] added()
{
  double gyrodirconst = this.gyrodir();
  double maxduetovel = this.maxduetovel();
  double[] duetovel = {Math.cos(gyrodirconst)*maxduetovel,Math.sin(gyrodirconst)*maxduetovel};
  double[][] rottovectors = this.rottovectors();
  double[][] holder = rottovectors;
  for(int i=0;i<rottovectors.length;i++)
  {
    holder[i] = new double[] {rottovectors[i][0]+duetovel[0],rottovectors[i][1]+duetovel[1]};
  }
  return holder;
}

public void drivebyjoystick()
{
  double[][] added = this.added();
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
