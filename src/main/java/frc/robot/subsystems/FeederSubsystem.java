// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  private final CANSparkMax feeder;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    this.feeder = new CANSparkMax(FeederConstants.feeder_ID, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopFeeder() {
    this.feeder.set(0.0);
  }

  public void setFeederSpeed() {
    double speedmotor = this.feeder.get();
    if (Math.abs(speedmotor) < FeederConstants.deadband) {
      double now = Timer.getFPGATimestamp();
      while (now < 0.6) {
        if (now == 0.2)
          this.feeder.set(FeederConstants.feederSpeed / 3);
        if (now == 0.4)
          this.feeder.set(FeederConstants.feederSpeed / 2);
      }
      this.feeder.set(FeederConstants.feederSpeed);
      // System.out.println("Timer : " + now);
    } else
      this.feeder.set(0);
  }
}
