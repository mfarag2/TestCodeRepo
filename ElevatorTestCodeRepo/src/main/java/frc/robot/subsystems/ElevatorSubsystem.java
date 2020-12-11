/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  public static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  /**
   * Creates a new ElevatorSubsystem.
   */
  // RobotController controller1 = new RobotController();

   private Encoder encoder1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
   private Encoder encoder2 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
   private TalonSRX motor1 = new TalonSRX(1);
   private TalonSRX motor2 new TalonSRX(2);

   public static double kP=0.0;
   public static double kI=0.0;
   public static double kD = 0.0;
   public static double kF = 0.0;

   public ElevatorSubsystem() {
     motor1.follow(motor2);
     // motor1.setProfile(0);
     // motor1.changeControlMode(TalonFXControlMode.MotionMagic);

     motor1.setSensorPhase(false);
     motor1.setInverted(false);

     motor1.config_kF(slotIdx, kF, timeoutMs);
     motor1.config_kP(slotIdx, kP, timeoutMs);
     motor1.config_kI(slotIdx, kI, timeoutMs);
     motor1.config_kD(slotIdx, kD, timeoutMs);

     motor1.configMotionCruiseVelocity(sensorUnitsPer100ms);
     motor1.configMotionAcceleration(sensorUnitsPer100msPerSec);

     // 0 the sensor 
     motor1.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

     
     /* Configure Current Limits */
     motor1.configPeakCurrentLimit(30); //Peak Current 25% motor Ma
     motor1.configPeakCurrentDuration(150);
     motor1.configContinuousCurrentLimit(20); //Stall current 15% motor Max
     // https://youtu.be/U4pgviiEwLg Citrus Circuits 1678, Presented by 973's Lead Mentor Adam Heard.

     private double gravityCompensation;
     arb_feedforward = gravityCompensation * horizontalHoldOutput; 
     motor1.set(ControlMode.MotionMagic, targetPos, DemandType.ArbitaryFeedForward, arb_feedforward);

     )

     /* public void setEncoderValue(){
      encoder1.setPosition(kF);
      encoder2.setPosition(kF);
  } */

  public 

  public void log(){
    SmartDashboard.putNumber("Encoder value",k);
    SmartDashboard.putNumber("P value",kP);
    SmartDashboard.putNumber("I value",kI);
    SmartDashboard.putNumber("D value",kD);
    SmartDashboard.putNumber("Elevator Position",motor1.getSelectedSensorPosition(0));

}

   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
