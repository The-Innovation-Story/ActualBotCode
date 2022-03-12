// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intaker;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    this.intaker = new CANSparkMax(IntakeConstants.intake_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Speed", this.intaker.get());
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    double speedmotor = this.intaker.get();
    if (Math.abs(speedmotor) < IntakeConstants.deadband) {
      double now = Timer.getFPGATimestamp();
      while (now < 0.6) {
        if (now == 0.2) this.intaker.set(speed / 3);
        if (now == 0.4) this.intaker.set(speed / 2);
      }
      this.intaker.set(speed);
      // System.out.println("Timer : " + now);
    }
    else this.intaker.set(0);
  }

}
