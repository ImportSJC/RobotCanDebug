package frc.robot.diagnostics;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.LinkedBlockingQueue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

/**
 * Passive CAN dropout diagnostics. Uses its own device objects (same IDs) so it never touches the
 * drivetrain's signals, never changes update frequencies, and only reads frames the devices already
 * broadcast. The one bus write is a single clearStickyFaults() per device at boot, after the boot
 * snapshot is recorded, so any sticky fault seen later happened during this run.
 *
 * Output: a plain-text log at /home/lvuser/logs/candiag/ (paste-ready) plus AdvantageKit outputs
 * under CanDiag/.
 */
public final class CanDropoutDiagnostics {
    private static final double STALE_SECONDS = 0.20;
    private static final double GAP_WARN_SECONDS = 0.060;
    private static final double HIGH_RES_WINDOW_SECONDS = 3.0;
    private static final int FAULT_POLL_DIVISOR = 5;
    private static final double ENABLED_STATUS_PERIOD = 1.0;
    private static final double DISABLED_STATUS_PERIOD = 5.0;

    private final List<Monitor> monitors = new ArrayList<>();
    private final CANBus bus;
    private final PowerDistribution pdh;
    private final LogWriter out;

    private final BusStats lifetime = new BusStats();
    private BusStats enableStats = new BusStats();
    private boolean wasEnabled = false;
    private double enableStart = 0;
    private int enableCount = 0;
    private long loop = 0;
    private double lastLoopTime = Double.NaN;
    private double lastStatusLine = -1e9;
    private double lastPhoenixBusPoll = -1e9;
    private CANStatus prevRio = null;
    private com.ctre.phoenix6.CANBus.CANBusStatus prevPhoenix = null;
    private PowerDistribution.StickyFaults prevPdhSticky = null;
    private PowerDistribution.Faults prevPdhFaults = null;
    private boolean prevBrownout = false;

    public record Device(String role, String kind, int id) {}

    public CanDropoutDiagnostics(CANBus bus, List<Device> devices, Map<String, String> config) {
        this.bus = bus;
        this.out = new LogWriter();
        out.line("==== CAN DROPOUT DIAGNOSTICS ====");
        out.line("wall clock: " + new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date())
                + " (roboRIO clock; may read 1970 until the DS connects)");
        out.line("bus: '" + bus.getName() + "' (" + (bus.getName().isEmpty() || bus.getName().equals("rio")
                ? "roboRIO built-in CAN 2.0" : "CANivore") + ")  isNetworkFD=" + bus.isNetworkFD());
        out.line("stale threshold " + ms(STALE_SECONDS) + "ms, gap warning " + ms(GAP_WARN_SECONDS) + "ms");
        config.forEach((k, v) -> out.line("config " + k + " = " + v));

        PowerDistribution p = null;
        try {
            p = new PowerDistribution();
            out.line("PDH/PDP: type=" + p.getType() + " module=" + p.getModule() + " channels=" + p.getNumChannels());
        } catch (RuntimeException e) {
            out.line("PDH/PDP: unavailable (" + e.getMessage() + ")");
        }
        pdh = p;

        for (Device d : devices) {
            Monitor m = switch (d.kind()) {
                case "CANcoder" -> Monitor.cancoder(d, bus);
                case "TalonFX" -> Monitor.talon(d, bus);
                case "Pigeon2" -> Monitor.pigeon(d, bus);
                default -> throw new IllegalArgumentException("Unknown kind " + d.kind());
            };
            monitors.add(m);
        }

        bootSnapshot();
        SmartDashboard.putString("CanDiag/Note", "");
        SmartDashboard.putBoolean("CanDiag/Mark", false);
        out.line("Type text into SmartDashboard 'CanDiag/Note' or toggle 'CanDiag/Mark' to stamp the log"
                + " (e.g. 'wiggling FL encoder plug').");
        out.line("");
        out.flush();
    }

    private void bootSnapshot() {
        out.line("---- boot snapshot (sticky faults from BEFORE this code started) ----");
        for (Monitor m : monitors) {
            BaseStatusSignal.waitForAll(0.25, m.allSignals());
            StringBuilder s = new StringBuilder(m.label()).append(": ");
            s.append(m.heartbeat.getStatus().isOK() ? "present" : "NOT RESPONDING (" + m.heartbeat.getStatus() + ")");
            s.append(", fw=").append(fw(m.version));
            if (m.supplyV != null) s.append(", Vsupply=").append(f2(m.supplyV.getValueAsDouble()));
            if (m.magnet != null) s.append(", magnet=").append(m.magnet.getValue());
            List<String> sticky = new ArrayList<>();
            m.sticky.forEach((name, sig) -> { if (Boolean.TRUE.equals(sig.getValue())) sticky.add(name); });
            s.append(", sticky=").append(sticky.isEmpty() ? "none" : String.join("|", sticky));
            out.line(s.toString());
            StatusCode clear = m.clearSticky.get();
            if (!clear.isOK()) out.line("  clearStickyFaults -> " + clear);
        }
        if (pdh != null) {
            PowerDistribution.StickyFaults sf = pdh.getStickyFaults();
            out.line("PDH sticky: " + pdhSticky(sf, pdh.getNumChannels()));
            pdh.clearStickyFaults();
        }
        out.line("sticky faults cleared on all devices; from here on a STICKY event means it happened this run.");
    }

    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double dt = Double.isNaN(lastLoopTime) ? 0 : now - lastLoopTime;
        lastLoopTime = now;
        loop++;
        boolean enabled = DriverStation.isEnabled();

        if (enabled && !wasEnabled) onEnable(now);
        if (!enabled && wasEnabled) onDisable(now);
        wasEnabled = enabled;

        double tEn = enabled ? now - enableStart : Double.NaN;
        String ctxTime = enabled ? String.format(Locale.US, "t=%.3f en+%.3fs", now, tEn)
                : String.format(Locale.US, "t=%.3f disabled", now);

        pollBus(now, ctxTime);
        pollRio(ctxTime);
        pollPdh(ctxTime);

        boolean pollFaults = loop % FAULT_POLL_DIVISOR == 0;
        for (Monitor m : monitors) {
            m.update(now, pollFaults, enabled, (msg) -> out.line("[" + ctxTime + "] " + msg + "  | " + context()),
                    enableStats.dev(m), lifetime.dev(m));
        }

        enableStats.loopDt(dt);
        lifetime.loopDt(dt);

        handleNotes(ctxTime);

        if (enabled && tEn <= HIGH_RES_WINDOW_SECONDS) highResLine(tEn, dt);
        double period = enabled ? ENABLED_STATUS_PERIOD : DISABLED_STATUS_PERIOD;
        if (now - lastStatusLine >= period) {
            lastStatusLine = now;
            statusLine(ctxTime);
        }

        recordAkit(enabled, dt);
        if (loop % 10 == 0) out.flush();
    }

    private void onEnable(double now) {
        enableCount++;
        enableStart = now;
        enableStats = new BusStats();
        for (Monitor m : monitors) m.resetEnableWindow();
        out.line("");
        out.line(String.format(Locale.US, "======== ENABLE #%d (%s) at t=%.3f  battery=%.2fV ========", enableCount,
                mode(), now, RobotController.getBatteryVoltage()));
        StringBuilder dis = new StringBuilder();
        for (Monitor m : monitors) if (!m.connected) dis.append(m.label()).append(" ");
        out.line("devices already missing at enable: " + (dis.length() == 0 ? "none" : dis.toString().trim()));
        out.line("high-res lines for first " + HIGH_RES_WINDOW_SECONDS
                + "s: en+t dt | util REC TEC txFull | batt | per-CANcoder msSinceFrame/Vsupply | per-steer supplyA");
    }

    private void onDisable(double now) {
        out.line(String.format(Locale.US, "======== DISABLE after ENABLE #%d, %.2fs enabled ========", enableCount,
                now - enableStart));
        summary("THIS ENABLE", enableStats, now - enableStart);
        summary("SINCE BOOT", lifetime, Double.NaN);
        out.line("");
        out.flush();
    }

    private void pollBus(double now, String ctxTime) {
        if (now - lastPhoenixBusPoll < 0.1) return;
        lastPhoenixBusPoll = now;
        var s = bus.getStatus();
        enableStats.phoenix(s);
        lifetime.phoenix(s);
        if (prevPhoenix != null) {
            if (s.BusOffCount != prevPhoenix.BusOffCount)
                out.line("[" + ctxTime + "] BUS-OFF (phoenix) count " + prevPhoenix.BusOffCount + " -> " + s.BusOffCount + "  | " + context());
            if (!s.Status.equals(prevPhoenix.Status))
                out.line("[" + ctxTime + "] Phoenix bus status " + prevPhoenix.Status + " -> " + s.Status);
            errorLevel(ctxTime, "phoenix REC", prevPhoenix.REC, s.REC);
            errorLevel(ctxTime, "phoenix TEC", prevPhoenix.TEC, s.TEC);
        }
        prevPhoenix = s;
    }

    private void pollRio(String ctxTime) {
        CANStatus c = RobotController.getCANStatus();
        enableStats.rio(c);
        lifetime.rio(c);
        if (prevRio != null) {
            if (c.busOffCount != prevRio.busOffCount)
                out.line("[" + ctxTime + "] BUS-OFF (rio) count " + prevRio.busOffCount + " -> " + c.busOffCount + "  | " + context());
            if (c.txFullCount - prevRio.txFullCount > 0 && enableStats.txFullEventsLogged++ < 50)
                out.line("[" + ctxTime + "] TX-FULL +" + (c.txFullCount - prevRio.txFullCount)
                        + " (rio transmit buffer full: bus too busy or nobody ACKing)  | " + context());
            errorLevel(ctxTime, "rio REC", prevRio.receiveErrorCount, c.receiveErrorCount);
            errorLevel(ctxTime, "rio TEC", prevRio.transmitErrorCount, c.transmitErrorCount);
        }
        prevRio = c;
        boolean bo = RobotController.isBrownedOut();
        if (bo != prevBrownout) {
            out.line("[" + ctxTime + "] roboRIO BROWNOUT " + (bo ? "START" : "END") + "  | " + context());
            if (bo) { enableStats.brownouts++; lifetime.brownouts++; }
        }
        prevBrownout = bo;
        double batt = RobotController.getBatteryVoltage();
        enableStats.minBattery = Math.min(enableStats.minBattery, batt);
        lifetime.minBattery = Math.min(lifetime.minBattery, batt);
    }

    private void errorLevel(String ctxTime, String name, int prev, int cur) {
        int a = level(prev), b = level(cur);
        if (a != b) {
            String[] names = {"ok(<96)", "WARNING(>=96)", "ERROR-PASSIVE(>=128)", "BUS-OFF(>=256)"};
            out.line("[" + ctxTime + "] " + name + " " + prev + " -> " + cur + " : " + names[a] + " -> " + names[b]
                    + "  | " + context());
        }
    }

    private static int level(int count) {
        return count >= 256 ? 3 : count >= 128 ? 2 : count >= 96 ? 1 : 0;
    }

    private void pollPdh(String ctxTime) {
        if (pdh == null || loop % 5 != 0) return;
        PowerDistribution.Faults f = pdh.getFaults();
        PowerDistribution.StickyFaults sf = pdh.getStickyFaults();
        int n = pdh.getNumChannels();
        if (prevPdhFaults != null) {
            String a = pdhFaults(prevPdhFaults, n), b = pdhFaults(f, n);
            if (!a.equals(b)) out.line("[" + ctxTime + "] PDH faults: " + a + " -> " + b);
            String sa = pdhSticky(prevPdhSticky, n), sb = pdhSticky(sf, n);
            if (!sa.equals(sb)) out.line("[" + ctxTime + "] PDH sticky: " + sa + " -> " + sb);
        }
        prevPdhFaults = f;
        prevPdhSticky = sf;
        double v = pdh.getVoltage();
        enableStats.minPdhV = Math.min(enableStats.minPdhV, v);
        lifetime.minPdhV = Math.min(lifetime.minPdhV, v);
        double total = pdh.getTotalCurrent();
        enableStats.maxPdhA = Math.max(enableStats.maxPdhA, total);
        lifetime.maxPdhA = Math.max(lifetime.maxPdhA, total);
        Logger.recordOutput("CanDiag/PDH/Voltage", v);
        Logger.recordOutput("CanDiag/PDH/TotalCurrent", total);
        Logger.recordOutput("CanDiag/PDH/ChannelCurrents", pdh.getAllCurrents());
    }

    private static String pdhFaults(PowerDistribution.Faults f, int n) {
        List<String> l = new ArrayList<>();
        if (f.Brownout) l.add("Brownout");
        if (f.CanWarning) l.add("CanWarning");
        for (int i = 0; i < n; i++) if (f.getBreakerFault(i)) l.add("Breaker" + i);
        return l.isEmpty() ? "none" : String.join("|", l);
    }

    private static String pdhSticky(PowerDistribution.StickyFaults f, int n) {
        List<String> l = new ArrayList<>();
        if (f.Brownout) l.add("Brownout");
        if (f.CanWarning) l.add("CanWarning");
        if (f.CanBusOff) l.add("CanBusOff");
        if (f.HasReset) l.add("HasReset");
        for (int i = 0; i < n; i++) if (f.getBreakerFault(i)) l.add("Breaker" + i);
        return l.isEmpty() ? "none" : String.join("|", l);
    }

    private String context() {
        StringBuilder s = new StringBuilder();
        if (prevRio != null) s.append(String.format(Locale.US, "util=%.0f%% REC=%d TEC=%d txFull=%d busOff=%d",
                prevRio.percentBusUtilization * 100, prevRio.receiveErrorCount, prevRio.transmitErrorCount,
                prevRio.txFullCount, prevRio.busOffCount));
        s.append(String.format(Locale.US, " batt=%.2fV", RobotController.getBatteryVoltage()));
        StringBuilder down = new StringBuilder();
        for (Monitor m : monitors) if (!m.connected) down.append(m.shortLabel()).append(',');
        s.append(" down=[").append(down.length() == 0 ? "" : down.substring(0, down.length() - 1)).append(']');
        return s.toString();
    }

    private void highResLine(double tEn, double dt) {
        StringBuilder s = new StringBuilder(String.format(Locale.US, "  hr en+%.3f dt=%2.0fms | ", tEn, dt * 1000));
        if (prevRio != null) s.append(String.format(Locale.US, "util=%3.0f%% REC=%3d TEC=%3d txF=%d | ",
                prevRio.percentBusUtilization * 100, prevRio.receiveErrorCount, prevRio.transmitErrorCount,
                prevRio.txFullCount));
        s.append(String.format(Locale.US, "batt=%.2f |", RobotController.getBatteryVoltage()));
        for (Monitor m : monitors) {
            if (m.kind.equals("CANcoder")) s.append(String.format(Locale.US, " %s:%s/%s", m.shortLabel(),
                    m.connected ? String.format(Locale.US, "%.0f", m.lastAge * 1000) : "DOWN",
                    m.supplyV == null ? "?" : f1(m.supplyV.getValueAsDouble())));
        }
        s.append(" |");
        for (Monitor m : monitors) {
            if (m.role.contains("steer") && m.supplyA != null)
                s.append(String.format(Locale.US, " %s:%.1fA", m.shortLabel(), m.supplyA.getValueAsDouble()));
        }
        out.line(s.toString());
    }

    private void statusLine(String ctxTime) {
        StringBuilder s = new StringBuilder("[" + ctxTime + "] status ").append(context());
        if (prevPhoenix != null) s.append(String.format(Locale.US, " phx(util=%.0f%% REC=%d TEC=%d txFull=%d)",
                prevPhoenix.BusUtilization * 100, prevPhoenix.REC, prevPhoenix.TEC, prevPhoenix.TxFullCount));
        if (pdh != null) s.append(String.format(Locale.US, " pdh=%.2fV/%.0fA", pdh.getVoltage(), pdh.getTotalCurrent()));
        s.append(" |");
        for (Monitor m : monitors) {
            if (!m.kind.equals("CANcoder")) continue;
            s.append(String.format(Locale.US, " %s %s maxGap=%.0fms V=%s", m.shortLabel(), m.connected ? "ok" : "DOWN",
                    m.windowMaxGap * 1000, m.supplyV == null ? "?" : f2(m.supplyV.getValueAsDouble())));
            m.windowMaxGap = 0;
        }
        out.line(s.toString());
    }

    private void summary(String title, BusStats st, double duration) {
        out.line("---- SUMMARY: " + title + (Double.isNaN(duration) ? "" : String.format(Locale.US, " (%.1fs)", duration)) + " ----");
        out.line(String.format(Locale.US,
                "bus rio: util avg=%.0f%% max=%.0f%% | REC max=%d TEC max=%d | txFull +%d | busOff +%d",
                st.utilAvg() * 100, st.maxUtil * 100, st.maxRec, st.maxTec, st.txFullDelta(), st.busOffDelta()));
        out.line(String.format(Locale.US,
                "bus phoenix: util max=%.0f%% | REC max=%d TEC max=%d | txFull +%d | busOff +%d",
                st.phxMaxUtil * 100, st.phxMaxRec, st.phxMaxTec, st.phxTxFullDelta(), st.phxBusOffDelta()));
        out.line(String.format(Locale.US,
                "power: battery min=%.2fV | rio brownouts=%d | pdh min=%.2fV max total=%.0fA | loop dt max=%.0fms overruns(>25ms)=%d",
                st.minBattery, st.brownouts, st.minPdhV, st.maxPdhA, st.maxDt * 1000, st.overruns));
        out.line(String.format(Locale.US, "%-26s %8s %9s %10s %10s %9s %9s  %s", "device", "dropouts", "down(s)",
                "maxGap(ms)", "gaps>60ms", "minV", "maxA", "faults seen"));
        boolean anyCoderDrop = false, anyCoderBoot = false, anyRemoteReset = false, anyRemoteInvalid = false,
                anyTalonDrop = false, anyUndervolt = false;
        for (Monitor m : monitors) {
            DevStats d = st.dev(m);
            out.line(String.format(Locale.US, "%-26s %8d %9.2f %10.0f %10d %9s %9s  %s", m.label(), d.dropouts,
                    d.downTime, d.maxGap * 1000, d.gapsOverWarn, Double.isInfinite(d.minV) ? "?" : f2(d.minV),
                    d.maxA < 0 ? "-" : f1(d.maxA), d.faults.isEmpty() ? "-" : String.join("|", d.faults)));
            if (m.kind.equals("CANcoder")) {
                anyCoderDrop |= d.dropouts > 0;
                anyCoderBoot |= d.faults.stream().anyMatch(x -> x.contains("BootDuringEnable"));
            } else if (d.dropouts > 0) anyTalonDrop = true;
            anyRemoteReset |= d.faults.stream().anyMatch(x -> x.contains("RemoteSensorReset"));
            anyRemoteInvalid |= d.faults.stream().anyMatch(x -> x.contains("RemoteSensorDataInvalid"));
            anyUndervolt |= d.faults.stream().anyMatch(x -> x.contains("Undervoltage"));
        }
        out.line("hints:");
        if (anyCoderBoot || anyRemoteReset)
            out.line("  * CANcoder REBOOTED (BootDuringEnable on a CANcoder or RemoteSensorReset on a steer motor)"
                    + " -> encoders lost POWER: check MPM, its feed/ground, crimps, connectors.");
        if (anyCoderDrop && !anyCoderBoot && !anyRemoteReset)
            out.line("  * CANcoders dropped WITHOUT rebooting -> COMMUNICATION problem: CAN wiring/connectors,"
                    + " termination, bus load. (Fault polling is 4Hz; a reboot shorter than that still shows as sticky.)");
        if (anyRemoteInvalid) out.line("  * Steer motors saw RemoteSensorDataInvalid -> they lost their CANcoder, which is the chatter.");
        if (anyTalonDrop) out.line("  * Talons dropped too -> problem is the shared bus, not just the encoders' power.");
        if (anyUndervolt) out.line("  * Undervoltage faults seen -> supply sagging at the device.");
        if (st.maxUtil > 0.80 || st.phxMaxUtil > 0.80) out.line("  * Bus utilization > 80% -> LOAD is a likely contributor.");
        if (st.maxRec >= 96 || st.maxTec >= 96 || st.phxMaxRec >= 96 || st.phxMaxTec >= 96)
            out.line("  * CAN error counters reached warning level -> ELECTRICAL bus errors (wiring/termination/noise).");
        if (st.txFullDelta() > 0) out.line("  * TX-full events -> rio couldn't transmit (load, or bus errors blocking ACK).");
        if (!anyCoderDrop && !anyTalonDrop) out.line("  * no dropouts detected in this window.");
    }

    private void handleNotes(String ctxTime) {
        String note = SmartDashboard.getString("CanDiag/Note", "");
        if (!note.isBlank()) {
            out.line("[" + ctxTime + "] NOTE: " + note + "  | " + context());
            SmartDashboard.putString("CanDiag/Note", "");
            out.flush();
        }
        if (SmartDashboard.getBoolean("CanDiag/Mark", false)) {
            out.line("[" + ctxTime + "] MARK  | " + context());
            SmartDashboard.putBoolean("CanDiag/Mark", false);
            out.flush();
        }
    }

    private void recordAkit(boolean enabled, double dt) {
        Logger.recordOutput("CanDiag/Enabled", enabled);
        Logger.recordOutput("CanDiag/LoopDtMs", dt * 1000);
        if (prevRio != null) {
            Logger.recordOutput("CanDiag/Rio/Utilization", prevRio.percentBusUtilization);
            Logger.recordOutput("CanDiag/Rio/REC", prevRio.receiveErrorCount);
            Logger.recordOutput("CanDiag/Rio/TEC", prevRio.transmitErrorCount);
            Logger.recordOutput("CanDiag/Rio/TxFullCount", prevRio.txFullCount);
            Logger.recordOutput("CanDiag/Rio/BusOffCount", prevRio.busOffCount);
        }
        if (prevPhoenix != null) {
            Logger.recordOutput("CanDiag/Phoenix/Utilization", prevPhoenix.BusUtilization);
            Logger.recordOutput("CanDiag/Phoenix/REC", prevPhoenix.REC);
            Logger.recordOutput("CanDiag/Phoenix/TEC", prevPhoenix.TEC);
            Logger.recordOutput("CanDiag/Phoenix/TxFullCount", prevPhoenix.TxFullCount);
            Logger.recordOutput("CanDiag/Phoenix/BusOffCount", prevPhoenix.BusOffCount);
        }
        Logger.recordOutput("CanDiag/BrownedOut", prevBrownout);
        for (Monitor m : monitors) {
            String k = "CanDiag/Devices/" + m.label().replace(' ', '_') + "/";
            Logger.recordOutput(k + "Connected", m.connected);
            Logger.recordOutput(k + "MsSinceFrame", m.lastAge * 1000);
            if (m.supplyV != null) Logger.recordOutput(k + "SupplyVoltage", m.supplyV.getValueAsDouble());
            if (m.supplyA != null) Logger.recordOutput(k + "SupplyCurrent", m.supplyA.getValueAsDouble());
            Logger.recordOutput(k + "ActiveFaults", m.activeFaults());
        }
        StringBuilder down = new StringBuilder();
        for (Monitor m : monitors) if (!m.connected) down.append(m.shortLabel()).append(' ');
        SmartDashboard.putString("CanDiag/Down", down.toString().trim());
    }

    private static String mode() {
        if (DriverStation.isAutonomous()) return "auto";
        if (DriverStation.isTest()) return "test";
        return "teleop";
    }

    private static String fw(StatusSignal<Integer> v) {
        if (!v.getStatus().isOK()) return "?";
        int x = v.getValue();
        return ((x >>> 24) & 0xff) + "." + ((x >>> 16) & 0xff) + "." + ((x >>> 8) & 0xff) + "." + (x & 0xff);
    }

    private static String f1(double v) { return String.format(Locale.US, "%.1f", v); }
    private static String f2(double v) { return String.format(Locale.US, "%.2f", v); }
    private static String ms(double s) { return String.format(Locale.US, "%.0f", s * 1000); }

    // ------------------------------------------------------------------------------------------

    private static final class Monitor {
        final String role, kind;
        final int id;
        final StatusSignal<?> heartbeat;
        final BaseStatusSignal supplyV;
        final BaseStatusSignal supplyA;
        final StatusSignal<MagnetHealthValue> magnet;
        final StatusSignal<Integer> version;
        final Map<String, StatusSignal<Boolean>> faults = new LinkedHashMap<>();
        final Map<String, StatusSignal<Boolean>> sticky = new LinkedHashMap<>();
        final java.util.function.Supplier<StatusCode> clearSticky;
        final BaseStatusSignal[] fast;
        final Map<String, Boolean> prevFault = new LinkedHashMap<>();
        final Map<String, Boolean> prevSticky = new LinkedHashMap<>();
        MagnetHealthValue prevMagnet = null;

        boolean connected = true;
        boolean everSeen = false;
        double downSince = 0;
        double lastFrameTs = 0;
        double lastAge = 0;
        double windowMaxGap = 0;

        private Monitor(Device d, StatusSignal<?> heartbeat, BaseStatusSignal supplyV, BaseStatusSignal supplyA,
                StatusSignal<MagnetHealthValue> magnet, StatusSignal<Integer> version,
                java.util.function.Supplier<StatusCode> clearSticky) {
            this.role = d.role();
            this.kind = d.kind();
            this.id = d.id();
            this.heartbeat = heartbeat;
            this.supplyV = supplyV;
            this.supplyA = supplyA;
            this.magnet = magnet;
            this.version = version;
            this.clearSticky = clearSticky;
            List<BaseStatusSignal> f = new ArrayList<>(List.of(heartbeat, supplyV));
            if (supplyA != null) f.add(supplyA);
            fast = f.toArray(BaseStatusSignal[]::new);
        }

        BaseStatusSignal[] slowSignals() {
            List<BaseStatusSignal> l = new ArrayList<>(faults.values());
            l.addAll(sticky.values());
            if (magnet != null) l.add(magnet);
            return l.toArray(BaseStatusSignal[]::new);
        }

        BaseStatusSignal[] allSignals() {
            List<BaseStatusSignal> l = new ArrayList<>(List.of(fast));
            l.addAll(List.of(slowSignals()));
            l.add(version);
            return l.toArray(BaseStatusSignal[]::new);
        }

        static Monitor cancoder(Device d, CANBus bus) {
            CANcoder c = new CANcoder(d.id(), bus);
            Monitor m = new Monitor(d, c.getAbsolutePosition(false), c.getSupplyVoltage(false), null,
                    c.getMagnetHealth(false), c.getVersion(false), c::clearStickyFaults);
            m.fault("Undervoltage", c.getFault_Undervoltage(false), c.getStickyFault_Undervoltage(false));
            m.fault("BootDuringEnable", c.getFault_BootDuringEnable(false), c.getStickyFault_BootDuringEnable(false));
            m.fault("Hardware", c.getFault_Hardware(false), c.getStickyFault_Hardware(false));
            m.fault("BadMagnet", c.getFault_BadMagnet(false), c.getStickyFault_BadMagnet(false));
            return m;
        }

        static Monitor talon(Device d, CANBus bus) {
            TalonFX t = new TalonFX(d.id(), bus);
            Monitor m = new Monitor(d, t.getPosition(false), t.getSupplyVoltage(false), t.getSupplyCurrent(false),
                    null, t.getVersion(false), t::clearStickyFaults);
            m.fault("Undervoltage", t.getFault_Undervoltage(false), t.getStickyFault_Undervoltage(false));
            m.fault("BootDuringEnable", t.getFault_BootDuringEnable(false), t.getStickyFault_BootDuringEnable(false));
            m.fault("BridgeBrownout", t.getFault_BridgeBrownout(false), t.getStickyFault_BridgeBrownout(false));
            m.fault("Hardware", t.getFault_Hardware(false), t.getStickyFault_Hardware(false));
            m.fault("RemoteSensorDataInvalid", t.getFault_RemoteSensorDataInvalid(false),
                    t.getStickyFault_RemoteSensorDataInvalid(false));
            m.fault("RemoteSensorReset", t.getFault_RemoteSensorReset(false), t.getStickyFault_RemoteSensorReset(false));
            m.fault("FusedSensorOutOfSync", t.getFault_FusedSensorOutOfSync(false),
                    t.getStickyFault_FusedSensorOutOfSync(false));
            return m;
        }

        static Monitor pigeon(Device d, CANBus bus) {
            Pigeon2 p = new Pigeon2(d.id(), bus);
            Monitor m = new Monitor(d, p.getYaw(false), p.getSupplyVoltage(false), null, null, p.getVersion(false),
                    p::clearStickyFaults);
            m.fault("Undervoltage", p.getFault_Undervoltage(false), p.getStickyFault_Undervoltage(false));
            m.fault("BootDuringEnable", p.getFault_BootDuringEnable(false), p.getStickyFault_BootDuringEnable(false));
            m.fault("Hardware", p.getFault_Hardware(false), p.getStickyFault_Hardware(false));
            return m;
        }

        void fault(String name, StatusSignal<Boolean> live, StatusSignal<Boolean> st) {
            faults.put(name, live);
            sticky.put(name, st);
        }

        String label() { return kind + " " + id + " (" + role + ")"; }
        String shortLabel() { return (kind.equals("CANcoder") ? "E" : kind.equals("Pigeon2") ? "P" : "M") + id; }

        String[] activeFaults() {
            List<String> l = new ArrayList<>();
            prevFault.forEach((k, v) -> { if (v) l.add(k); });
            return l.toArray(String[]::new);
        }

        void resetEnableWindow() { windowMaxGap = 0; }

        void update(double now, boolean pollFaults, boolean enabled, java.util.function.Consumer<String> log,
                DevStats en, DevStats life) {
            BaseStatusSignal.refreshAll(fast);
            double ts = heartbeat.getTimestamp().getTime();
            boolean ok = heartbeat.getStatus().isOK();
            if (ok && ts != lastFrameTs) {
                if (lastFrameTs > 0) {
                    double gap = ts - lastFrameTs;
                    windowMaxGap = Math.max(windowMaxGap, gap);
                    en.gap(gap);
                    life.gap(gap);
                    if (gap > GAP_WARN_SECONDS && gap <= STALE_SECONDS && en.gapLines++ < 40)
                        log.accept(String.format(Locale.US, "GAP %s %.0fms between frames (not a full dropout)", label(), gap * 1000));
                }
                lastFrameTs = ts;
                everSeen = true;
            }
            lastAge = everSeen ? Math.max(0, now - lastFrameTs) : Double.POSITIVE_INFINITY;
            boolean nowConnected = ok && everSeen && lastAge <= STALE_SECONDS;
            if (supplyV != null && nowConnected) {
                en.voltage(supplyV.getValueAsDouble());
                life.voltage(supplyV.getValueAsDouble());
            }
            if (supplyA != null && nowConnected) {
                en.current(supplyA.getValueAsDouble());
                life.current(supplyA.getValueAsDouble());
            }

            if (connected && !nowConnected) {
                downSince = now;
                en.dropouts++;
                life.dropouts++;
                log.accept(String.format(Locale.US, "DROPOUT %s: last frame %.0fms ago, status=%s, lastV=%s",
                        label(), lastAge * 1000, heartbeat.getStatus(),
                        supplyV == null ? "?" : f2(supplyV.getValueAsDouble())));
            } else if (!connected && nowConnected) {
                double down = now - downSince;
                en.downTime += down;
                life.downTime += down;
                log.accept(String.format(Locale.US, "RECOVER %s after %.3fs, V=%s", label(), down,
                        supplyV == null ? "?" : f2(supplyV.getValueAsDouble())));
            }
            connected = nowConnected;

            if (!pollFaults) return;
            BaseStatusSignal.refreshAll(slowSignals());
            for (var e : faults.entrySet()) {
                if (!e.getValue().getStatus().isOK()) continue;
                boolean v = Boolean.TRUE.equals(e.getValue().getValue());
                Boolean p = prevFault.put(e.getKey(), v);
                if (p != null && p != v) {
                    log.accept("FAULT " + (v ? "SET" : "cleared") + " " + label() + " " + e.getKey());
                    if (v) { en.faults.add(e.getKey()); life.faults.add(e.getKey()); }
                }
            }
            for (var e : sticky.entrySet()) {
                if (!e.getValue().getStatus().isOK()) continue;
                boolean v = Boolean.TRUE.equals(e.getValue().getValue());
                Boolean p = prevSticky.put(e.getKey(), v);
                if (p != null && !p && v) {
                    log.accept("STICKY SET " + label() + " " + e.getKey());
                    en.faults.add("sticky:" + e.getKey());
                    life.faults.add("sticky:" + e.getKey());
                }
            }
            if (magnet != null && magnet.getStatus().isOK()) {
                MagnetHealthValue mh = magnet.getValue();
                if (prevMagnet != null && mh != prevMagnet) log.accept("MAGNET " + label() + " " + prevMagnet + " -> " + mh);
                prevMagnet = mh;
            }
        }
    }

    private static final class DevStats {
        int dropouts, gapsOverWarn, gapLines;
        double downTime, maxGap, minV = Double.POSITIVE_INFINITY, maxA = -1;
        final java.util.Set<String> faults = new java.util.LinkedHashSet<>();

        void gap(double g) {
            maxGap = Math.max(maxGap, g);
            if (g > GAP_WARN_SECONDS) gapsOverWarn++;
        }
        void voltage(double v) { if (v > 0.5) minV = Math.min(minV, v); }
        void current(double a) { maxA = Math.max(maxA, a); }
    }

    private static final class BusStats {
        final Map<Monitor, DevStats> devs = new java.util.IdentityHashMap<>();
        double utilSum, maxUtil, phxMaxUtil, minBattery = Double.POSITIVE_INFINITY, minPdhV = Double.POSITIVE_INFINITY,
                maxPdhA, maxDt;
        long utilN;
        int maxRec, maxTec, phxMaxRec, phxMaxTec, brownouts, overruns, txFullEventsLogged;
        int firstTxFull = -1, lastTxFull, firstBusOff = -1, lastBusOff;
        int phxFirstTxFull = -1, phxLastTxFull, phxFirstBusOff = -1, phxLastBusOff;

        DevStats dev(Monitor m) { return devs.computeIfAbsent(m, k -> new DevStats()); }

        void rio(CANStatus c) {
            utilSum += c.percentBusUtilization;
            utilN++;
            maxUtil = Math.max(maxUtil, c.percentBusUtilization);
            maxRec = Math.max(maxRec, c.receiveErrorCount);
            maxTec = Math.max(maxTec, c.transmitErrorCount);
            if (firstTxFull < 0) { firstTxFull = c.txFullCount; firstBusOff = c.busOffCount; }
            lastTxFull = c.txFullCount;
            lastBusOff = c.busOffCount;
        }

        void phoenix(com.ctre.phoenix6.CANBus.CANBusStatus s) {
            if (!s.Status.isOK()) return;
            phxMaxUtil = Math.max(phxMaxUtil, s.BusUtilization);
            phxMaxRec = Math.max(phxMaxRec, s.REC);
            phxMaxTec = Math.max(phxMaxTec, s.TEC);
            if (phxFirstTxFull < 0) { phxFirstTxFull = s.TxFullCount; phxFirstBusOff = s.BusOffCount; }
            phxLastTxFull = s.TxFullCount;
            phxLastBusOff = s.BusOffCount;
        }

        void loopDt(double dt) {
            maxDt = Math.max(maxDt, dt);
            if (dt > 0.025) overruns++;
        }

        double utilAvg() { return utilN == 0 ? 0 : utilSum / utilN; }
        int txFullDelta() { return firstTxFull < 0 ? 0 : lastTxFull - firstTxFull; }
        int busOffDelta() { return firstBusOff < 0 ? 0 : lastBusOff - firstBusOff; }
        int phxTxFullDelta() { return phxFirstTxFull < 0 ? 0 : phxLastTxFull - phxFirstTxFull; }
        int phxBusOffDelta() { return phxFirstBusOff < 0 ? 0 : phxLastBusOff - phxFirstBusOff; }
    }

    /** Writes on a background thread so file I/O never stalls the robot loop. */
    private static final class LogWriter {
        private final LinkedBlockingQueue<String> queue = new LinkedBlockingQueue<>(20000);
        private final File file;

        LogWriter() {
            File dir = new File(RobotBase.isReal() ? "/home/lvuser/logs/candiag" : "logs/candiag");
            dir.mkdirs();
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            file = new File(dir, "candiag_" + stamp + "_" + System.nanoTime() % 100000 + ".txt");
            Thread t = new Thread(this::run, "CanDiagWriter");
            t.setDaemon(true);
            t.start();
            System.out.println("[CanDiag] writing " + file.getAbsolutePath());
            SmartDashboard.putString("CanDiag/File", file.getAbsolutePath());
        }

        void line(String s) { queue.offer(s); }

        void flush() { queue.offer("\u0000FLUSH"); }

        private void run() {
            try (BufferedWriter w = new BufferedWriter(new FileWriter(file, true))) {
                while (true) {
                    String s = queue.take();
                    if (s.equals("\u0000FLUSH")) w.flush();
                    else { w.write(s); w.newLine(); }
                }
            } catch (IOException | InterruptedException e) {
                DriverStation.reportError("[CanDiag] log writer stopped: " + e, false);
            }
        }
    }
}
