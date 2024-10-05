// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    //change id
    private final LaserCan m_laser = new LaserCan(1);;
    static boolean intaking = false;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(80);
    }

    public void set(double speed) {
        intakeMotor.set(speed);
    }

    public void intake() {
        intakeMotor.set(Constants.Intake.intakingSpeed);
    }

    public void eject() {
        intakeMotor.set(Constants.Intake.ejectingSpeed);
    }

    /**
     * Stops both wheels.
     */
    public void stop() {
        intakeMotor.set(0);
    }

    public static boolean hasNote(LaserCan laser) {
        LaserCan.Measurement measurement = laser.getMeasurement();
        if (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 2.0) {
            return true;
        }
        return false;
   }

   /*public void runIntake() {
    if(hasNote(m_laser) == true){
        intakeMotor.set(0.0);
    }
    else{
        intakeMotor.set(Constants.Intake.intakingSpeed);
    }
  }*/

    /*@Override
    public void periodic() {
        double current = intakeMotor.getTorqueCurrent().getValueAsDouble();
        boolean active = (current > 20.0);

        if (active && !intaking) {
        }
        SmartDashboard.putNumber("intake/torqueCurrent", current);

    }*/
}