// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FeatureSwitches;
import frc.robot.diagnostics.CanDropoutDiagnostics;
import frc.robot.generated.TunerConstants;
import frc.robot.util.GamePeriod;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private HootAutoReplay m_timeAndJoystickReplay;

    private CanDropoutDiagnostics m_canDiagnostics;

    // TODO(2027): Verify autonomous duration for 2027 game rules (was 20.0s in 2026).
    private static final double AUTO_DURATION = 20.0;
    private static Timer autoTimer = new Timer();

    public Robot() {
        // ── AdvantageKit setup — must run before any other initialization ──────
        Logger.recordMetadata("ProjectName", "2027Robot");
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            // Check only the explicit AKIT_LOG_PATH env var — NOT the AdvantageScope temp file.
            // findReplayLog() checks both, which caused any open AS log to silently hijack
            // simulateJava into replay mode (no NT4Publisher → no NT entries). Now:
            //   • simulateJava with no env var  → live sim + NT (normal dev workflow)
            //   • ./gradlew replayWatch         → sets AKIT_LOG_PATH before calling simulateJava
            //   • AKIT_LOG_PATH=path simulateJava → explicit one-shot replay
            String logPath = System.getenv("AKIT_LOG_PATH");
            if (logPath != null) {
                setUseTiming(false);
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            } else {
                Logger.addDataReceiver(new NT4Publisher());
            }
        }

        Logger.start();

        // CTRE Hoot replay — initialized after Logger.start() per AKit guidance
        m_timeAndJoystickReplay = new HootAutoReplay()
                .withTimestampReplay()
                .withJoystickReplay();

        if (RobotBase.isReal()) {
            DataLogManager.start("/home/lvuser/logs");
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        m_robotContainer = new RobotContainer();

        var modules = m_robotContainer.drivetrain.getModules();
        if (!FeatureSwitches.SKIP_SWERVE_POWER_TELEMETRY) {
            for (int i = 0; i < modules.length; i++) {
                frc.robot.power.PowerTelemetry.register(modules[i].getDriveMotor(), "Drive " + i, "Drivetrain");
                frc.robot.power.PowerTelemetry.register(modules[i].getSteerMotor(), "Steer " + i, "Drivetrain");
            }
        }
        // Declarations let replay consume new power inputs without creating mechanism hardware.
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.INTAKE_MOTOR, "Intake Deploy", "Intake");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.PICKUP_MOTOR, "Pickup", "Pickup");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.HOPPER_MOTOR, "Hopper", "Hopper");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.INDEXER_MOTOR, "Indexer", "Indexer");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.CLIMBER_MOTOR, "Climber Climber", "Climber");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.CLIMBER_GRABBER, "Climber Grabber", "Climber");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.SHOOTER_MOTOR_1, "Shooter Motor1", "Shooter");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.SHOOTER_MOTOR_2, "Shooter Motor2", "Shooter");
        frc.robot.power.PowerTelemetry.declare("rio", Constants.CANBus.SHOOTER_MOTOR_3, "Shooter Motor3", "Shooter");
        if (!FeatureSwitches.DISABLE_POWER_TELEMETRY) frc.robot.power.PowerTelemetry.initialize();

        if (FeatureSwitches.CAN_DROPOUT_DIAGNOSTICS && System.getenv("AKIT_LOG_PATH") == null) {
            m_canDiagnostics = createCanDiagnostics(modules);
        }

        GamePeriod.elasticInit();
    }

    private static CanDropoutDiagnostics createCanDiagnostics(
            SwerveModule<TalonFX, TalonFX, CANcoder>[] modules) {
        String[] names = {"FL", "FR", "BL", "BR"};
        List<CanDropoutDiagnostics.Device> devices = new ArrayList<>();
        for (int i = 0; i < modules.length; i++) {
            String n = i < names.length ? names[i] : "M" + i;
            devices.add(new CanDropoutDiagnostics.Device(n + " encoder", "CANcoder", modules[i].getEncoder().getDeviceID()));
        }
        for (int i = 0; i < modules.length; i++) {
            String n = i < names.length ? names[i] : "M" + i;
            devices.add(new CanDropoutDiagnostics.Device(n + " steer", "TalonFX", modules[i].getSteerMotor().getDeviceID()));
            devices.add(new CanDropoutDiagnostics.Device(n + " drive", "TalonFX", modules[i].getDriveMotor().getDeviceID()));
        }
        devices.add(new CanDropoutDiagnostics.Device("gyro", "Pigeon2", TunerConstants.DrivetrainConstants.Pigeon2Id));
        devices.add(new CanDropoutDiagnostics.Device("intake deploy", "TalonFX", Constants.CANBus.INTAKE_MOTOR));
        devices.add(new CanDropoutDiagnostics.Device("pickup", "TalonFX", Constants.CANBus.PICKUP_MOTOR));
        devices.add(new CanDropoutDiagnostics.Device("hopper", "TalonFX", Constants.CANBus.HOPPER_MOTOR));
        devices.add(new CanDropoutDiagnostics.Device("indexer", "TalonFX", Constants.CANBus.INDEXER_MOTOR));
        devices.add(new CanDropoutDiagnostics.Device("climber", "TalonFX", Constants.CANBus.CLIMBER_MOTOR));
        devices.add(new CanDropoutDiagnostics.Device("climber grabber", "TalonFX", Constants.CANBus.CLIMBER_GRABBER));
        devices.add(new CanDropoutDiagnostics.Device("shooter 1", "TalonFX", Constants.CANBus.SHOOTER_MOTOR_1));
        devices.add(new CanDropoutDiagnostics.Device("shooter 2", "TalonFX", Constants.CANBus.SHOOTER_MOTOR_2));
        devices.add(new CanDropoutDiagnostics.Device("shooter 3", "TalonFX", Constants.CANBus.SHOOTER_MOTOR_3));

        Map<String, String> config = new LinkedHashMap<>();
        config.put("DISABLE_POWER_TELEMETRY", String.valueOf(FeatureSwitches.DISABLE_POWER_TELEMETRY));
        config.put("SKIP_SWERVE_POWER_TELEMETRY", String.valueOf(FeatureSwitches.SKIP_SWERVE_POWER_TELEMETRY));
        config.put("DISABLE_INTAKE", String.valueOf(FeatureSwitches.DISABLE_INTAKE));
        config.put("DISABLE_INDEXER", String.valueOf(FeatureSwitches.DISABLE_INDEXER));
        config.put("DISABLE_HOPPER", String.valueOf(FeatureSwitches.DISABLE_HOPPER));
        config.put("odometryFrequencyHz", String.valueOf(TunerConstants.kCANBus.isNetworkFD() ? 250 : 100));
        return new CanDropoutDiagnostics(TunerConstants.kCANBus, devices, config);
    }

    private void resetSubsystems_init() {
        CommandScheduler.getInstance().schedule(m_robotContainer.indexer.runStopIndexer());
        CommandScheduler.getInstance().schedule(m_robotContainer.hopper.runStopHopper());
    }

    private void resetSubsystems_disable() {
        CommandScheduler.getInstance().schedule(m_robotContainer.shooter.stopShooter());
        CommandScheduler.getInstance().schedule(m_robotContainer.indexer.runStopIndexer());
        CommandScheduler.getInstance().schedule(m_robotContainer.hopper.runStopHopper());
    }

    private static void startAutoTimer() {
        autoTimer.reset();
        autoTimer.start();
    }

    @Override
    public void robotPeriodic() {
        var d = m_canDiagnostics;
        if (d != null) d.beginLoop();
        m_timeAndJoystickReplay.update();
        m_robotContainer.correctOdometry();
        if (d != null) d.lap("hootReplay+odometry");
        CommandScheduler.getInstance().run();
        if (d != null) d.lap("scheduler");
        if (!FeatureSwitches.DISABLE_POWER_TELEMETRY) frc.robot.power.PowerTelemetry.periodic();
        if (d != null) d.lap("powerTelemetry");
        RobotContainer.updateNT();
        RobotContainer.publishRobotData();
        if (d != null) d.lap("nt+robotData");
        m_robotContainer.drivetrain.publishDriveOutputVoltage();
        m_robotContainer.drivetrain.publishMotorCurrent();
        m_robotContainer.drivetrain.publishDrivePidErrors();
        m_robotContainer.drivetrain.publishDistanceToHub();
        m_robotContainer.intake.publishMotorCurrents();
        if (d != null) d.lap("dashboardPublish");
        if (d != null) d.periodic();
    }

    @Override
    public void disabledInit() {
        if (RobotBase.isReal()) {
            DataLogManager.getLog().flush();
        }
        resetSubsystems_disable();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        resetSubsystems_init();
        startAutoTimer();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        GamePeriod.elasticTeleopInit();
        resetSubsystems_init();
    }

    @Override
    public void teleopPeriodic() {
        GamePeriod.elasticPeriodic();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        // IO sim classes manage their own physics — no global sim runner needed.
    }

    public static double getAutonomousTimeLeft() {
        double fmsTime = Timer.getMatchTime();
        if (fmsTime >= 0) return fmsTime;
        return Math.max(AUTO_DURATION - autoTimer.get(), 0);
    }
}
