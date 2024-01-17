// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found below.

// MIT License
//
// Copyright (c) 2023 FRC 6328
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

package swervelib.telemetry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

/**
 * Class for managing persistent alerts to be sent over NetworkTables.
 */
public class Alert
{

  private static Map<String, SendableAlerts> groups = new HashMap<String, SendableAlerts>();

  private final AlertType type;
  private       boolean   active          = false;
  private       double    activeStartTime = 0.0;
  private       String    text;

  /**
   * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated, the appropriate
   * entries will be added to NetworkTables.
   *
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  public Alert(String text, AlertType type)
  {
    this("Alerts", text, type);
  }

  /**
   * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate entries will be added to
   * NetworkTables.
   *
   * @param group Group identifier, also used as NetworkTables title
   * @param text  Text to be displayed when the alert is active.
   * @param type  Alert level specifying urgency.
   */
  public Alert(String group, String text, AlertType type)
  {
    if (!groups.containsKey(group))
    {
      groups.put(group, new SendableAlerts());
      SmartDashboard.putData(group, groups.get(group));
    }

    this.text = text;
    this.type = type;
    groups.get(group).alerts.add(this);
  }

  /**
   * Sets whether the alert should currently be displayed. When activated, the alert text will also be sent to the
   * console.
   */
  public void set(boolean active)
  {
    if (active && !this.active)
    {
      activeStartTime = Timer.getFPGATimestamp();
      switch (type)
      {
        case ERROR:
          DriverStation.reportError(text, false);
          break;
        case ERROR_TRACE:
          DriverStation.reportError(text, true);
          break;
        case WARNING:
          DriverStation.reportWarning(text, false);
          break;
        case WARNING_TRACE:
          DriverStation.reportWarning(text, true);
          break;
        case INFO:
          System.out.println(text);
          break;
      }
    }
    this.active = active;
  }

  /**
   * Updates current alert text.
   */
  public void setText(String text)
  {
    if (active && !text.equals(this.text))
    {
      switch (type)
      {
        case ERROR:
          DriverStation.reportError(text, false);
          break;
        case ERROR_TRACE:
          DriverStation.reportError(text, true);
          break;
        case WARNING:
          DriverStation.reportWarning(text, false);
          break;
        case WARNING_TRACE:
          DriverStation.reportWarning(text, true);
          break;
        case INFO:
          System.out.println(text);
          break;
      }
    }
    this.text = text;
  }

  /**
   * Represents an alert's level of urgency.
   */
  public static enum AlertType
  {
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type for problems which
     * will seriously affect the robot's functionality and thus require immediate attention.
     */
    ERROR,
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type for problems which
     * will seriously affect the robot's functionality and thus require immediate attention. Trace printed to driver
     * station console.
     */
    ERROR_TRACE,

    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this type for problems
     * which could affect the robot's functionality but do not necessarily require immediate attention.
     */
    WARNING,
    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this type for problems
     * which could affect the robot's functionality but do not necessarily require immediate attention. Trace printed to
     * driver station console.
     */
    WARNING_TRACE,
    /**
     * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type for problems which
     * are unlikely to affect the robot's functionality, or any other alerts which do not fall under "ERROR" or
     * "WARNING".
     */
    INFO
  }

  private static class SendableAlerts implements Sendable
  {

    public final List<Alert> alerts = new ArrayList<>();

    public String[] getStrings(AlertType type)
    {
      Predicate<Alert> activeFilter = (Alert x) -> x.type == type && x.active;
      Comparator<Alert> timeSorter =
          (Alert a1, Alert a2) -> (int) (a2.activeStartTime - a1.activeStartTime);
      return alerts.stream()
                   .filter(activeFilter)
                   .sorted(timeSorter)
                   .map((Alert a) -> a.text)
                   .toArray(String[]::new);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
      builder.setSmartDashboardType("Alerts");
      builder.addStringArrayProperty("errors", () -> getStrings(AlertType.ERROR), null);
      builder.addStringArrayProperty("errors", () -> getStrings(AlertType.ERROR_TRACE), null);
      builder.addStringArrayProperty("warnings", () -> getStrings(AlertType.WARNING), null);
      builder.addStringArrayProperty("warnings", () -> getStrings(AlertType.WARNING_TRACE), null);
      builder.addStringArrayProperty("infos", () -> getStrings(AlertType.INFO), null);
    }
  }
}