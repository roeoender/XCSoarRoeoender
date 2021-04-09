/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "GlueMapWindow.hpp"
#include "Look/MapLook.hpp"
#include "Screen/Icon.hpp"
#include "Language/Language.hpp"
#include "Screen/Layout.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Engine/Task/TaskManager.hpp"
#include "Engine/Task/Ordered/OrderedTask.hpp"
#include "Renderer/TextInBox.hpp"
#include "Weather/Rasp/RaspRenderer.hpp"
#include "Formatter/UserUnits.hpp"
#include "Formatter/UserGeoPointFormatter.hpp"
#include "UIState.hpp"
#include "Renderer/FinalGlideBarRenderer.hpp"
#include "Terrain/RasterTerrain.hpp"
#include "Util/Macros.hpp"
#include "Util/Clamp.hpp"
#include "Util/StringAPI.hxx"
#include "Look/GestureLook.hpp"
#include "Input/InputEvents.hpp"
#include "Renderer/MapScaleRenderer.hpp"

#include <stdio.h>

// jarek
#include "Engine/Waypoint/Waypoints.hpp"
#include "Engine/GlideSolvers/MacCready.hpp"
#include "Task/RoutePlannerGlue.hpp"
#include "Engine/Route/RoutePlanner.hpp"
#include "Engine/GlideSolvers/GlideState.hpp"
#include "Interface.hpp"
#include "MainWindow.hpp"
#include "Units/Units.hpp"
#include "Formatter/AngleFormatter.hpp"
#include "Computer/GlideComputer.hpp"
#include "Airspace/NearestAirspace.hpp"
#include "Engine/Airspace/AbstractAirspace.hpp"
#include "Util/TruncateString.hpp"

// Shift of base center coordinate of values overlay
#define OVR_OFFSET 47
// wysokosc jednego napisu
#define OVR_SHIFT 30

extern Airspaces airspace_database;

void
GlueMapWindow::DrawGesture(Canvas &canvas) const
{
  if (!gestures.HasPoints())
    return;

  const TCHAR *gesture = gestures.GetGesture();
  if (gesture != nullptr && !InputEvents::IsGesture(gesture))
    canvas.Select(gesture_look.invalid_pen);
  else
    canvas.Select(gesture_look.pen);

  canvas.SelectHollowBrush();

  const auto &points = gestures.GetPoints();
  auto it = points.begin();
  auto it_last = it++;
  for (auto it_end = points.end(); it != it_end; it_last = it++)
    canvas.DrawLinePiece(*it_last, *it);
}

void
GlueMapWindow::DrawCrossHairs(Canvas &canvas) const
{
  if (!render_projection.IsValid())
    return;

  canvas.Select(look.overlay.crosshair_pen);

  const auto center = render_projection.GetScreenOrigin();

  canvas.DrawLine(center.x + 20, center.y,
              center.x - 20, center.y);
  canvas.DrawLine(center.x, center.y + 20,
              center.x, center.y - 20);
}

void
GlueMapWindow::DrawPanInfo(Canvas &canvas) const
{
  if (!render_projection.IsValid())
    return;

  GeoPoint location = render_projection.GetGeoLocation();

  TextInBoxMode mode;
  mode.shape = LabelShape::OUTLINED;
  mode.align = TextInBoxMode::Alignment::RIGHT;

  const Font &font = *look.overlay.overlay_font;
  canvas.Select(font);

  unsigned padding = Layout::FastScale(4);
  unsigned height = font.GetHeight();
  int y = 0 + padding;
  int x = render_projection.GetScreenWidth() - padding;

  if (compass_visible)
    /* don't obscure the north arrow */
    /* TODO: obtain offset from CompassRenderer */
    y += Layout::Scale(19) + Layout::FastScale(13);

  if (terrain) {
    TerrainHeight elevation = terrain->GetTerrainHeight(location);
    if (!elevation.IsSpecial()) {
      StaticString<64> elevation_long;
      elevation_long = _("Elevation: ");
      elevation_long += FormatUserAltitude(elevation.GetValue());

      TextInBox(canvas, elevation_long, x, y, mode,
                render_projection.GetScreenWidth(),
                render_projection.GetScreenHeight());

      y += height;
    }
  }

  TCHAR buffer[256];
  FormatGeoPoint(location, buffer, ARRAY_SIZE(buffer), _T('\n'));

  TCHAR *start = buffer;
  while (true) {
    auto *newline = StringFind(start, _T('\n'));
    if (newline != nullptr)
      *newline = _T('\0');

    TextInBox(canvas, start, x, y, mode,
              render_projection.GetScreenWidth(),
              render_projection.GetScreenHeight());

    y += height;

    if (newline == nullptr)
      break;

    start = newline + 1;
  }
}

// bazje na FormatLabel i CalculateReachabilityDirect
double
GlueMapWindow::CalculateHomeArrival(
  const WaypointPtr &home_waypoint,
  const PolarSettings &polar_settings,
  const TaskBehaviour &task_behaviour) const {
  if (!Basic().location_available || !Basic().NavAltitudeAvailable()) {
    return 0.0;
  }
  const auto elevation = home_waypoint->elevation + task_behaviour.safety_height_arrival;

  const GlidePolar &glide_polar = task_behaviour.route_planner.reach_polar_mode == RoutePlannerConfig::Polar::TASK
  ? polar_settings.glide_polar_task
  : Calculated().glide_polar_safety;
  
  const MacCready mac_cready(task_behaviour.glide, glide_polar);
  const SpeedVector &wind = Calculated().GetWindOrZero();
  const GlideState state(GeoVector(Basic().location, home_waypoint->location),
    elevation, Basic().nav_altitude, wind);
  
  const GlideResult result = mac_cready.SolveStraight(state);
  if (!result.IsOk()) {
    return 0.0;
  }
  return result.pure_glide_altitude_difference;
}

#define OVR_STR_BUF_SIZE 50

void
GlueMapWindow::DrawValuesOverlay(Canvas &canvas, const PixelRect &rc) const
{
 // home arrival
 const ComputerSettings &settings_computer = GetComputerSettings();
 const TaskBehaviour &task_behaviour = settings_computer.task;

  TextInBoxMode mode;
  mode.shape = LabelShape::ROUNDED_BLACK;
  const Font &font = *look.overlay.overlay_value_font;
  const Font &font2 = *look.overlay.overlay_medium_font;
  const Font &font3 = *look.overlay.overlay_font;

  canvas.Select(font);
  TCHAR valChar[OVR_STR_BUF_SIZE];
  StaticString<80> buffer;

  int y_base_center = (rc.bottom + rc.top) / 2 - Layout::FastScale(OVR_OFFSET);

  // ----- altitudes - left center
    PixelPoint p(rc.left + Layout::FastScale(2),
                 y_base_center);
  {
    buffer.clear();

    auto hAgl = Calculated().altitude_agl;
    FormatUserAltitude(hAgl, valChar, false);
    buffer += valChar;

    buffer += _T("|");

    auto hGps = Basic().gps_altitude_available ? (int)Basic().gps_altitude : 0;
    FormatUserAltitude(hGps, valChar, false);
    buffer += valChar;
    bool smaller_font = false;
    // when the text is long we use smaller font
    if (buffer.length() > 7) {
      smaller_font = true;
      canvas.Select(font3);
    }
    TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
    if (smaller_font) {
      canvas.Select(font);
    }
  }

  //---------- right center  true airspeed and ground speed
  {
    buffer.clear();
    if (Basic().airspeed_available) {
      auto ts = Basic().true_airspeed;
      FormatUserSpeed(ts, valChar, false, false);
      buffer += valChar;
    }
    if (Basic().ground_speed_available) {
      buffer += _T("|");
      auto bv = Basic().ground_speed;
      FormatUserSpeed(bv, valChar, false, false);
      buffer += valChar;
    }
    const PixelSize text_size = canvas.CalcTextSize(buffer);

    p.x = rc.right - text_size.cx;
    p.y = y_base_center;

    TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
  }

  // font outlined yellow used from here
  mode.shape = LabelShape::OUTLINED_YELLOW;
  canvas.Select(font2);

  //--------- right lower-center: MacCready setting
  {
    buffer.clear();

    const double mc = settings_computer.polar.glide_polar_task.GetMC();
    FormatUserVerticalSpeed(mc, valChar, false, false);
    buffer += valChar;
    buffer += settings_computer.task.auto_mc ? _("A") : _("M");
    const PixelSize text_size = canvas.CalcTextSize(buffer);
    p.x = rc.right - text_size.cx;
    p.y = y_base_center + Layout::FastScale(25);
    TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
  }
  // prawe centrom-gora: predkosc dolphin/block
  {
    buffer.clear();
    if (settings_computer.features.block_stf_enabled) {
      FormatUserSpeed(Calculated().common_stats.V_block, valChar, false, false);
      buffer += valChar;
      buffer += _("|");
    }
    FormatUserSpeed(Calculated().common_stats.V_dolphin, valChar, false, false);
    buffer += valChar;

    const PixelSize text_size = canvas.CalcTextSize(buffer);

    p.x = rc.right - text_size.cx;
    p.y = y_base_center - Layout::FastScale(OVR_SHIFT);
    TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);

  }
  //--------- lewe centrum-dol: nawigacja
  if (terrain != nullptr) {
    p.x = rc.left + Layout::FastScale(2);
    p.y = y_base_center + Layout::FastScale(25); // so nonstandard
    auto elevation = terrain->GetTerrainHeight(Basic().location);
    if (!elevation.IsSpecial()) {
      FormatUserAltitude(elevation.GetValue(), valChar, false);
      buffer.clear();
      buffer += valChar;
      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
    }
  }
  //--------- left upper-center: vertical airspace
  {
    NearestAirspace nearest = NearestAirspace::FindVertical(
      Basic(),
      Calculated(),
      glide_computer->GetAirspaceWarnings(),
      airspace_database
//      airspace_renderer.GetAirspaces()
    );
    if (!nearest.IsDefined()) {
      buffer.clear();
      // buffer += valChar;
      buffer += _T("---");
      // just debug: FormatUserSpeed(airspace_renderer.GetAirspaces()->GetSize(), valChar, false, false);
      // buffer += valChar;
      p.x = rc.left + Layout::FastScale(2);
      p.y = y_base_center - Layout::FastScale(OVR_SHIFT);

      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
    } else {
      buffer.clear();
      FormatRelativeUserAltitude(nearest.distance, valChar, false);
      buffer += valChar;
      p.x = rc.left + Layout::FastScale(2);
      p.y = y_base_center - Layout::FastScale(OVR_SHIFT);
      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);

      buffer.clear();
      buffer += nearest.airspace->GetName();
      p.x = rc.left + Layout::FastScale(40);
      p.y = rc.top + Layout::FastScale(25 + 20);


      canvas.Select(font3);
      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
      canvas.Select(font2);
    }
  }

  // next waypoint
  if (task != nullptr) {
    const WaypointPtr way_point = task->GetActiveWaypoint();
    if (way_point != nullptr) {
      buffer.clear();
      // const TCHAR *value = way_point->name.c_str();
      CopyTruncateString(valChar, OVR_STR_BUF_SIZE, way_point->name.c_str(), 7);
      buffer += valChar;
      buffer += _T(" : ");
      const TaskStats &task_stats = Calculated().task_stats;
      // altitude arrival
      {
        const auto &next_solution = task_stats.current_leg.solution_remaining;
        if (
          Basic().NavAltitudeAvailable() &&
          task_stats.task_valid &&
          next_solution.IsAchievable()) {
          // if you want Alt difference:
          // auto altitude_difference = next_solution.SelectAltitudeDifference(computer_settings.task.glide);
          // FormatRelativeUserAltitude(altitude_difference, valChar, false);
          auto waypt_arrival_alt = next_solution.GetArrivalAltitude(Basic().nav_altitude);
          FormatUserAltitude((int)Units::ToUserAltitude(waypt_arrival_alt), valChar, false);
          buffer += valChar;
        }
      }

      p.x = rc.left + Layout::FastScale(40),
      p.y = rc.top + Layout::FastScale(0);
      canvas.Select(font2);
      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);

      // distance & direction
      buffer.clear();
      const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
      if (task_stats.task_valid && vector_remaining.IsValid()) {
        FormatUserDistanceSmart(vector_remaining.distance, valChar, false);
        buffer += valChar;
        if (Basic().track_available) {
          buffer += _T(" : ");
          Angle bd = vector_remaining.bearing - Basic().track;
          FormatAngleDelta(valChar,10, bd);
          buffer += valChar;
        }
      }

      p.x = rc.left + Layout::FastScale(40),
      p.y = rc.top + Layout::FastScale(20);
      canvas.Select(font2);
      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
    }
  }

  // home waypoint
  auto waypoints = CommonInterface::main_window->GetMap()->waypoints;
  if (waypoints != nullptr && waypoints->GetHome() != nullptr) {
    auto home_waypoint = waypoints->GetHome();
    if (home_waypoint != nullptr) {
      buffer.clear();
      CopyTruncateString(valChar, OVR_STR_BUF_SIZE, home_waypoint->name.c_str(), 7);
      buffer += valChar;
      // home arrival altitude
      buffer += _T(" : ");
      double home_arrival_height = CalculateHomeArrival(home_waypoint, settings_computer.polar, task_behaviour);
      FormatUserAltitude((int)Units::ToUserAltitude(home_arrival_height), valChar, false);
      buffer += valChar;
      p.x = (rc.left + rc.right) / 2 - Layout::FastScale(40),
      p.y = rc.bottom - Layout::FastScale(40);
      canvas.Select(font2);
      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);

      // dystans i kierunek
      buffer.clear();
      if (Calculated().common_stats.vector_home.IsValid()) {
        double distance_home = Calculated().common_stats.vector_home.distance;
        FormatUserDistanceSmart(distance_home, valChar, false);
        buffer += valChar;
        buffer += _T(" : ");
        if (Basic().track_available) {
          Angle angle_home = Calculated().common_stats.vector_home.bearing - Basic().track;
          FormatAngleDelta(valChar,10, angle_home);
          buffer += valChar;
        }
      }
      p.x = (rc.left + rc.right) / 2 - Layout::FastScale(40),
      p.y = rc.bottom - Layout::FastScale(20);
      canvas.Select(font2);
      TextInBox(canvas, buffer, p.x, p.y, mode, rc, nullptr);
    }
  }

}

void
GlueMapWindow::DrawGPSStatus(Canvas &canvas, const PixelRect &rc,
                             const NMEAInfo &info) const
{
  const TCHAR *txt;
  const MaskedIcon *icon;

  if (!info.alive) {
    icon = &look.no_gps_icon;
    txt = _("GPS not connected");
  } else if (!info.location_available) {
    icon = &look.waiting_for_fix_icon;
    txt = _("GPS waiting for fix");
  } else
    // early exit
    return;

  PixelPoint p(rc.left + Layout::FastScale(2),
               rc.bottom - Layout::FastScale(35));
  icon->Draw(canvas, p);

  p.x += icon->GetSize().cx + Layout::FastScale(4);
  p.y = rc.bottom - Layout::FastScale(34);

  TextInBoxMode mode;
  mode.shape = LabelShape::ROUNDED_BLACK;

  const Font &font = *look.overlay.overlay_font;
  canvas.Select(font);
  TextInBox(canvas, txt, p.x, p.y, mode, rc, nullptr);
}

void
GlueMapWindow::DrawFlightMode(Canvas &canvas, const PixelRect &rc) const
{
  int offset = 0;

  // draw flight mode
  const MaskedIcon *bmp;

  if (Calculated().common_stats.task_type == TaskType::ABORT)
    bmp = &look.abort_mode_icon;
  else if (GetDisplayMode() == DisplayMode::CIRCLING)
    bmp = &look.climb_mode_icon;
  else if (GetDisplayMode() == DisplayMode::FINAL_GLIDE)
    bmp = &look.final_glide_mode_icon;
  else
    bmp = &look.cruise_mode_icon;

  offset += bmp->GetSize().cx + Layout::Scale(6);

  bmp->Draw(canvas,
            PixelPoint(rc.right - offset,
                       rc.bottom - bmp->GetSize().cy - Layout::Scale(4)));

  // draw flarm status
  if (!GetMapSettings().show_flarm_alarm_level)
    // Don't show indicator when the gauge is indicating the traffic anyway
    return;

  const FlarmStatus &flarm = Basic().flarm.status;
  if (!flarm.available)
    return;

  switch (flarm.alarm_level) {
  case FlarmTraffic::AlarmType::NONE:
    bmp = &look.traffic_safe_icon;
    break;
  case FlarmTraffic::AlarmType::LOW:
  case FlarmTraffic::AlarmType::INFO_ALERT:
    bmp = &look.traffic_warning_icon;
    break;
  case FlarmTraffic::AlarmType::IMPORTANT:
  case FlarmTraffic::AlarmType::URGENT:
    bmp = &look.traffic_alarm_icon;
    break;
  };

  offset += bmp->GetSize().cx + Layout::Scale(6);

  bmp->Draw(canvas,
            PixelPoint(rc.right - offset,
                       rc.bottom - bmp->GetSize().cy - Layout::Scale(2)));
}

void
GlueMapWindow::DrawFinalGlide(Canvas &canvas, const PixelRect &rc) const
{

  if (GetMapSettings().final_glide_bar_display_mode==FinalGlideBarDisplayMode::OFF)
    return;

  if (GetMapSettings().final_glide_bar_display_mode==FinalGlideBarDisplayMode::AUTO) {
    const TaskStats &task_stats = Calculated().task_stats;
    const ElementStat &total = task_stats.total;
    const GlideResult &solution = total.solution_remaining;
    const GlideResult &solution_mc0 = total.solution_mc0;
    const GlideSettings &glide_settings= GetComputerSettings().task.glide;

    if (!task_stats.task_valid || !solution.IsOk() || !solution_mc0.IsDefined())
      return;

    if (solution_mc0.SelectAltitudeDifference(glide_settings) < -1000 &&
        solution.SelectAltitudeDifference(glide_settings) < -1000)
      return;
  }

  final_glide_bar_renderer.Draw(canvas, rc, Calculated(),
                                GetComputerSettings().task.glide,
                                GetMapSettings().final_glide_bar_mc0_enabled);
}

void
GlueMapWindow::DrawVario(Canvas &canvas, const PixelRect &rc) const
{
  if (!GetMapSettings().vario_bar_enabled)
   return;

  vario_bar_renderer.Draw(canvas, rc, Basic(), Calculated(),
                                GetComputerSettings().polar.glide_polar_task,
                                true); //NOTE: AVG enabled for now, make it configurable ;
}

void
GlueMapWindow::DrawMapScale(Canvas &canvas, const PixelRect &rc,
                            const MapWindowProjection &projection) const
{
  RenderMapScale(canvas, projection, rc, look.overlay);

  if (!projection.IsValid())
    return;

  StaticString<80> buffer;

  buffer.clear();

  if (GetMapSettings().auto_zoom_enabled)
    buffer = _T("AUTO ");

  switch (follow_mode) {
  case FOLLOW_SELF:
    break;

  case FOLLOW_PAN:
    buffer += _T("PAN ");
    break;
  }

  const UIState &ui_state = GetUIState();
  if (ui_state.auxiliary_enabled) {
    buffer += ui_state.panel_name;
    buffer += _T(" ");
  }

  if (Basic().gps.replay)
    buffer += _T("REPLAY ");
  else if (Basic().gps.simulator) {
    buffer += _("SIM");
    buffer += _T(" ");
  }

  if (GetComputerSettings().polar.ballast_timer_active)
    buffer.AppendFormat(
        _T("BALLAST %d LITERS "),
        (int)GetComputerSettings().polar.glide_polar_task.GetBallastLitres());

  if (rasp_renderer != nullptr) {
    const TCHAR *label = rasp_renderer->GetLabel();
    if (label != nullptr)
      buffer += gettext(label);
  }

  if (!buffer.empty()) {

    const Font &font = *look.overlay.overlay_font;
    canvas.Select(font);
    const unsigned height = font.GetCapitalHeight()
        + Layout::GetTextPadding();
    int y = rc.bottom - height;

    TextInBoxMode mode;
    mode.vertical_position = TextInBoxMode::VerticalPosition::ABOVE;
    mode.shape = LabelShape::OUTLINED;

    TextInBox(canvas, buffer, 0, y, mode, rc, nullptr);
  }
}

void
GlueMapWindow::DrawThermalEstimate(Canvas &canvas) const
{
  if (InCirclingMode() && IsNearSelf()) {
    // in circling mode, draw thermal at actual estimated location
    const MapWindowProjection &projection = render_projection;
    const ThermalLocatorInfo &thermal_locator = Calculated().thermal_locator;
    if (thermal_locator.estimate_valid) {
      PixelPoint sc;
      if (projection.GeoToScreenIfVisible(thermal_locator.estimate_location, sc)) {
        look.thermal_source_icon.Draw(canvas, sc);
      }
    }
  } else {
    MapWindow::DrawThermalEstimate(canvas);
  }
}

void
GlueMapWindow::RenderTrail(Canvas &canvas, const PixelPoint aircraft_pos)
{
  unsigned min_time;
  switch(GetMapSettings().trail.length) {
  case TrailSettings::Length::OFF:
    return;
  case TrailSettings::Length::LONG:
    min_time = std::max(0, (int)Basic().time - 3600);
    break;
  case TrailSettings::Length::SHORT:
    min_time = std::max(0, (int)Basic().time - 600);
    break;
  case TrailSettings::Length::FULL:
  default:
    min_time = 0; // full
    break;
  }

  DrawTrail(canvas, aircraft_pos, min_time,
            GetMapSettings().trail.wind_drift_enabled && InCirclingMode());
}

void
GlueMapWindow::RenderTrackBearing(Canvas &canvas, const PixelPoint aircraft_pos)
{
  DrawTrackBearing(canvas, aircraft_pos, InCirclingMode());
}

void
GlueMapWindow::DrawThermalBand(Canvas &canvas, const PixelRect &rc) const
{
  if (Calculated().task_stats.total.solution_remaining.IsOk() &&
      Calculated().task_stats.total.solution_remaining.altitude_difference > 50
      && GetDisplayMode() == DisplayMode::FINAL_GLIDE)
    return;

  PixelRect tb_rect;
  tb_rect.left = rc.left;
  tb_rect.right = rc.left+Layout::Scale(25);
  tb_rect.top = Layout::Scale(2);
  tb_rect.bottom = (rc.bottom-rc.top)/5 - Layout::Scale(2);

  const ThermalBandRenderer &renderer = thermal_band_renderer;
  if (task != nullptr) {
    ProtectedTaskManager::Lease task_manager(*task);
    renderer.DrawThermalBand(Basic(),
                             Calculated(),
                             GetComputerSettings(),
                             canvas,
                             tb_rect,
                             GetComputerSettings().task,
                             true,
                             &task_manager->GetOrderedTask().GetOrderedTaskSettings());
  } else {
    renderer.DrawThermalBand(Basic(),
                             Calculated(),
                             GetComputerSettings(),
                             canvas,
                             tb_rect,
                             GetComputerSettings().task,
                             true);
  }
}

void
GlueMapWindow::DrawStallRatio(Canvas &canvas, const PixelRect &rc) const
{
  if (Basic().stall_ratio_available) {
    // JMW experimental, display stall sensor
    auto s = Clamp(Basic().stall_ratio, 0., 1.);
    int m = rc.GetHeight() * s * s;

    canvas.SelectBlackPen();
    canvas.DrawLine(rc.right - 1, rc.bottom - m, rc.right - 11, rc.bottom - m);
  }
}
