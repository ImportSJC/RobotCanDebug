 *  Executing task: gradlew build   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk" 


> Task :compileJava FAILED
C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:28: error: cannot find symbol
import edu.wpi.first.wpilibj.PowerDistributionFaults;
                            ^
  symbol:   class PowerDistributionFaults
  location: package edu.wpi.first.wpilibj
C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:29: error: cannot find symbol
import edu.wpi.first.wpilibj.PowerDistributionStickyFaults;
                            ^
  symbol:   class PowerDistributionStickyFaults
  location: package edu.wpi.first.wpilibj
C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:70: error: cannot find symbol
    private PowerDistributionStickyFaults prevPdhSticky = null;
            ^
  symbol:   class PowerDistributionStickyFaults
  location: class CanDropoutDiagnostics
C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:71: error: cannot find symbol
    private PowerDistributionFaults prevPdhFaults = null;
            ^
  symbol:   class PowerDistributionFaults
  location: class CanDropoutDiagnostics
C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:283: error: cannot find symbol
    private static String pdhFaults(PowerDistributionFaults f, int n) {
                                    ^
  symbol:   class PowerDistributionFaults
  location: class CanDropoutDiagnostics
C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:291: error: cannot find symbol
    private static String pdhSticky(PowerDistributionStickyFaults f, int n) {
                                    ^
  symbol:   class PowerDistributionStickyFaults
  location: class CanDropoutDiagnostics
6 errors
Compilation Error!
GradleRIO detected this build failed due to a Compile Error (compileJava).
Check that all your files are saved, then scroll up in this log for more information.
[Incubating] Problems report is available at: file:///C:/Users/FinneyRobotics/VS%20Code/RobotCanDebug/build/reports/problems/problems-report.html

FAILURE: Build failed with an exception.

* What went wrong:
Execution failed for task ':compileJava'.
> Compilation failed; see the compiler output below.
  C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:28: error: cannot find symbol
  import edu.wpi.first.wpilibj.PowerDistributionFaults;
                              ^
    symbol:   class PowerDistributionFaults
    location: package edu.wpi.first.wpilibj
  C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:29: error: cannot find symbol
  import edu.wpi.first.wpilibj.PowerDistributionStickyFaults;
                              ^
    symbol:   class PowerDistributionStickyFaults
    location: package edu.wpi.first.wpilibj
  C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:70: error: cannot find symbol
      private PowerDistributionStickyFaults prevPdhSticky = null;
              ^
    symbol:   class PowerDistributionStickyFaults
    location: class CanDropoutDiagnostics
  C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:71: error: cannot find symbol
      private PowerDistributionFaults prevPdhFaults = null;
              ^
    symbol:   class PowerDistributionFaults
    location: class CanDropoutDiagnostics
  C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:283: error: cannot find symbol
      private static String pdhFaults(PowerDistributionFaults f, int n) {
                                      ^
    symbol:   class PowerDistributionFaults
    location: class CanDropoutDiagnostics
  C:\Users\FinneyRobotics\VS Code\RobotCanDebug\src\main\java\frc\robot\diagnostics\CanDropoutDiagnostics.java:291: error: cannot find symbol
      private static String pdhSticky(PowerDistributionStickyFaults f, int n) {
                                      ^
    symbol:   class PowerDistributionStickyFaults
    location: class CanDropoutDiagnostics
  6 errors

* Try:
> Check your code and dependencies to fix the compilation error(s)
> Run with --scan to get full insights.

BUILD FAILED in 3s
1 actionable task: 1 executed

 *  The terminal process "cmd.exe /d /c gradlew build   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"" terminated with exit code: 1. 
 *  Terminal will be reused by tasks, press any key to close it. 