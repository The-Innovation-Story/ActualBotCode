// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final WPI_TalonFX falcon;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.falcon = new WPI_TalonFX(ClimberConstants.falcon_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFalconSpeed() {
    double speedmotor = this.falcon.get();
    if (Math.abs(speedmotor) < ClimberConstants.deadband) {
      double now = Timer.getFPGATimestamp();
      while (now < 0.6) {
        if (now == 0.2)
          this.falcon.set(ClimberConstants.falconSpeed / 3);
        if (now == 0.4)
          this.falcon.set(ClimberConstants.falconSpeed / 2);
      }
      this.falcon.set(ClimberConstants.falconSpeed);
      // System.out.println("Timer : " + now);
    } else
      this.falcon.set(0);
  }
}
