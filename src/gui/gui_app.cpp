#include "gui_app.hpp"

#include "osm2xodr/model_builder.hpp"
#include "osm2xodr/osm_parse.hpp"
#include "osm2xodr/report.hpp"
#include "osm2xodr/xodr_writer.hpp"

#include <imgui.h>
#include <tinyfiledialogs.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>

namespace osm2xodr::gui {

namespace {
constexpr float kToolbarHeight = 36.0f;
constexpr float kSplitterWidth = 6.0f;
constexpr double kMarkingDashM = 3.0; // dash/gap length in world meters, matching test/xodr_viewer.py
constexpr double kMarkingGapM = 3.0;

// Swapped from the original red-centerline/yellow-edges scheme: centerline is now yellow, lane
// edges are now red.
const ImU32 kColorCenter = IM_COL32(255, 212, 0, 230);
const ImU32 kColorLane = IM_COL32(255, 80, 80, 200);
const ImU32 kColorMarking = IM_COL32(235, 235, 235, 220);

double rad_to_deg(double r) { return r * 180.0 / geo::kPi; }

std::string filename_only(const std::string& path) {
    const size_t sep = path.find_last_of("/\\");
    return sep == std::string::npos ? path : path.substr(sep + 1);
}

// Same directory and base name as the input, with a .xodr extension -- strips a recognized OSM
// extension (including the doubled ".osm.pbf"/".osm.bz2" some extracts use) rather than just the
// last '.' segment, so "karlsruhe.osm.pbf" becomes "karlsruhe.xodr", not "karlsruhe.osm.xodr".
std::string derive_output_path(const std::string& input_path) {
    const size_t sep = input_path.find_last_of("/\\");
    const std::string dir = (sep == std::string::npos) ? std::string() : input_path.substr(0, sep + 1);
    const std::string filename = filename_only(input_path);

    static const std::vector<std::string> kOsmExtensions = {
        ".osm.pbf", ".osm.bz2", ".osm.gz", ".osm.xml", ".pbf", ".osm", ".xml",
    };
    std::string stem = filename;
    for (const auto& ext : kOsmExtensions) {
        if (stem.size() > ext.size() && stem.compare(stem.size() - ext.size(), ext.size(), ext) == 0) {
            stem.erase(stem.size() - ext.size());
            break;
        }
    }
    if (stem == filename) {
        // No recognized OSM extension -- fall back to stripping whatever the last '.' segment is.
        const size_t dot = stem.find_last_of('.');
        if (dot != std::string::npos && dot > 0) stem.erase(dot);
    }
    return dir + stem + ".xodr";
}

// Native "Open" dialog (tinyfiledialogs: Cocoa/AppleScript on macOS, zenity/kdialog on Linux,
// the Win32 common dialogs on Windows). Blocking, like any native modal file picker -- the whole
// app pauses while it's open, same as the existing blocking WMS fetch during Convert.
void open_input_file(Session& session) {
    const char* filters[] = {"*.osm", "*.pbf", "*.osm.pbf", "*.osm.bz2"};
    const char* result = tinyfd_openFileDialog("Open OSM file", "", 4, filters,
                                                "OpenStreetMap files (*.osm, *.pbf)", 0);
    if (!result) return; // cancelled
    session.options.input = result;
    session.options.output = derive_output_path(result);
    session.label = filename_only(result);
}

void save_output_file(Session& session) {
    const char* filters[] = {"*.xodr"};
    const char* result = tinyfd_saveFileDialog("Save XODR file", session.options.output.c_str(), 1,
                                                filters, "OpenDRIVE files (*.xodr)");
    if (!result) return; // cancelled
    session.options.output = result;
}

// ImGui::InputText needs a fixed, writable char buffer; these small helpers copy a std::string
// in, edit in place, and copy the (possibly changed) result back out.
bool input_text(const char* label, std::string& value, size_t buf_size = 512) {
    std::vector<char> buf(buf_size, '\0');
    std::snprintf(buf.data(), buf_size, "%s", value.c_str());
    if (ImGui::InputText(label, buf.data(), buf_size)) {
        value = buf.data();
        return true;
    }
    return false;
}

// Index ranges of pts (a roughly-uniformly-sampled world-space polyline, in meters) that fall in
// the 'on' part of a dash/gap pattern measured by cumulative arc length, so the dash length stays
// a constant number of meters regardless of sampling density or zoom -- same approach as
// test/xodr_viewer.py's dash_runs().
std::vector<std::pair<int, int>> dash_index_ranges(const std::vector<WorldPoint>& pts,
                                                     double dash = kMarkingDashM, double gap = kMarkingGapM) {
    std::vector<std::pair<int, int>> runs;
    if (pts.size() < 2) return runs;
    std::vector<double> cum(pts.size(), 0.0);
    for (std::size_t i = 1; i < pts.size(); ++i)
        cum[i] = cum[i - 1] + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);

    const double period = dash + gap;
    int start = -1;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const bool on = std::fmod(cum[i], period) < dash;
        if (on && start < 0) {
            start = static_cast<int>(i);
        } else if (!on && start >= 0) {
            if (static_cast<int>(i) - start >= 2) runs.emplace_back(start, static_cast<int>(i));
            start = -1;
        }
    }
    if (start >= 0 && static_cast<int>(pts.size()) - start >= 2) runs.emplace_back(start, static_cast<int>(pts.size()));
    return runs;
}
} // namespace

void Camera::fit(double min_x, double min_y, double max_x, double max_y, float margin) {
    const double w = std::max(max_x - min_x, 1e-3);
    const double h = std::max(max_y - min_y, 1e-3);
    cx = static_cast<float>((min_x + max_x) / 2.0);
    cy = static_cast<float>((min_y + max_y) / 2.0);
    scale = static_cast<float>(std::min(width / (w * margin), height / (h * margin)));
}

GuiApp::GuiApp() { add_session(); }

GuiApp::~GuiApp() {
    for (auto& session : sessions_) release_tile(session.imagery);
}

void GuiApp::add_session() {
    Session s;
    s.label = "Conversion " + std::to_string(++session_counter_);
    sessions_.push_back(std::move(s));
    active_session_ = static_cast<int>(sessions_.size()) - 1;
}

void GuiApp::close_session(int index) {
    if (sessions_.size() <= 1 || index < 0 || index >= static_cast<int>(sessions_.size())) return;
    release_tile(sessions_[index].imagery);
    sessions_.erase(sessions_.begin() + index);
    if (active_session_ >= static_cast<int>(sessions_.size()))
        active_session_ = static_cast<int>(sessions_.size()) - 1;
}

void GuiApp::fetch_imagery(Session& session) {
    release_tile(session.imagery);
    session.imagery = ImageryTile{};
    session.imagery_status.clear();

    if (!session.has_origin) {
        session.imagery_status = "No projection origin available -- imagery skipped.";
        return;
    }

    const auto& g = session.geometry;
    constexpr double kMaxRadiusM = 2000.0;
    const double half_w = (g.max_x - g.min_x) / 2.0;
    const double half_h = (g.max_y - g.min_y) / 2.0;
    const double wanted_radius = std::max(half_w, half_h) * 1.15;
    const double radius_m = std::clamp(wanted_radius, 30.0, kMaxRadiusM);
    const double center_x = (g.min_x + g.max_x) / 2.0;
    const double center_y = (g.min_y + g.max_y) / 2.0;

    // Inverse of osm2xodr::geo::LocalProjector::project: local meters -> lat/lon, centered on
    // the network's own bounding box rather than the raw <geoReference> origin (which may sit
    // off in a corner) -- see lgl_imagery.hpp's fetch_tile() docstring for why local_origin_lat/
    // lon must still be the *road* origin regardless of where the fetch itself is centered.
    const double cos_lat0 = std::cos(session.origin_lat * geo::kPi / 180.0);
    const double center_lon = session.origin_lon + rad_to_deg(center_x / (geo::kEarthRadiusM * cos_lat0));
    const double center_lat = session.origin_lat + rad_to_deg(center_y / geo::kEarthRadiusM);

    session.imagery = gui::fetch_tile(center_lat, center_lon, radius_m, session.origin_lat, session.origin_lon, "dop20");
    // A freshly uploaded texture always has the unadjusted pixels baked in; reset so
    // draw_preview_panel()'s lazy brightness/contrast sync notices it needs to reapply the
    // current global sliders (if they're non-default) rather than assuming a match by coincidence.
    session.applied_brightness = 0;
    session.applied_contrast = 1.0f;
    if (!session.imagery.error.empty()) {
        session.imagery_status = session.imagery.error;
    } else {
        session.imagery_status = std::to_string(session.imagery.width_px) + "x" +
                                  std::to_string(session.imagery.height_px) + "px";
        if (wanted_radius > kMaxRadiusM) {
            session.imagery_status += "  (network extends beyond fetched imagery, capped at " +
                                       std::to_string(static_cast<int>(kMaxRadiusM)) + "m radius)";
        }
    }
}

void GuiApp::run_conversion(Session& session) {
    session.log.clear();
    if (session.options.input.empty() || session.options.output.empty()) {
        session.status = "Set both an input file and an output file before converting.";
        return;
    }
    try {
        auto parsed = osm::parse_osm(session.options);
        auto model = build::build_model(parsed, session.options);
        xodr::write_file(model, session.options);
        write_report(model, session.options, parsed);

        session.last_road_count = static_cast<int>(model.roads.size());
        session.last_junction_count = static_cast<int>(model.junctions.size());
        session.status = "Wrote " + session.options.output + " with " +
                          std::to_string(session.last_road_count) + " road(s) and " +
                          std::to_string(session.last_junction_count) + " junction(s).";
        if (!model.warnings.empty()) {
            session.status += "  (" + std::to_string(model.warnings.size()) + " warning(s), see log below)";
            for (const auto& w : model.warnings) {
                session.log += w;
                session.log += '\n';
            }
        }

        session.geometry = build_preview_geometry(model, 2.0);
        session.origin_lat = model.projector.origin.lat;
        session.origin_lon = model.projector.origin.lon;
        session.has_origin = model.projector.has_origin;
        session.camera_initialized = false;
        fetch_imagery(session);
    } catch (const std::exception& e) {
        session.last_road_count = -1;
        session.last_junction_count = -1;
        session.status = std::string("Conversion failed: ") + e.what();
    }
}

void GuiApp::draw_command_bar() {
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New Tab")) add_session();
            if (ImGui::MenuItem("Close Tab", nullptr, false, sessions_.size() > 1))
                pending_close_ = active_session_;
            ImGui::Separator();
            if (ImGui::MenuItem("Open OSM...", nullptr, false, !sessions_.empty()))
                open_input_file(sessions_[active_session_]);
            if (ImGui::MenuItem("Convert", nullptr, false, !sessions_.empty()))
                run_conversion(sessions_[active_session_]);
            if (ImGui::MenuItem("Save XODR As...", nullptr, false, !sessions_.empty()))
                save_output_file(sessions_[active_session_]);
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) quit_requested_ = true;
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("About osm2xodr")) show_about_ = true;
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    if (show_about_) {
        ImGui::OpenPopup("About osm2xodr");
        show_about_ = false;
    }
    if (ImGui::BeginPopupModal("About osm2xodr", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::Text("osm2xodr GUI prototype");
        ImGui::Text("Dear ImGui %s", IMGUI_VERSION);
        ImGui::Spacing();
        ImGui::TextDisabled("The command-line osm2xodr binary is unaffected by this GUI.");
        if (ImGui::Button("Close")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

void GuiApp::draw_toolbar() {
    ImGui::BeginChild("Toolbar", ImVec2(0, kToolbarHeight), true,
                       ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    if (!sessions_.empty()) {
        Session& session = sessions_[active_session_];
        if (ImGui::Button("Open...")) open_input_file(session);
        ImGui::SameLine(0.0f, 16.0f);
        if (ImGui::Button("Convert")) run_conversion(session);

        ImGui::SameLine(0.0f, 20.0f);
        ImGui::TextDisabled("|");

        ImGui::SameLine(0.0f, 20.0f);
        ImGui::Checkbox("Imagery", &show_imagery_);
        ImGui::SameLine(0.0f, 12.0f);
        ImGui::Checkbox("Centerline", &show_center_);
        ImGui::SameLine(0.0f, 12.0f);
        ImGui::Checkbox("Lane edges", &show_lanes_);
        ImGui::SameLine(0.0f, 12.0f);
        ImGui::Checkbox("Markings", &show_markings_);

        ImGui::SameLine(0.0f, 20.0f);
        ImGui::TextDisabled("|");

        ImGui::SameLine(0.0f, 20.0f);
        ImGui::Text("Brightness");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f);
        ImGui::SliderInt("##brightness", &brightness_, -150, 150);

        ImGui::SameLine(0.0f, 16.0f);
        ImGui::Text("Contrast");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f);
        ImGui::SliderFloat("##contrast", &contrast_, 0.2f, 3.0f, "%.1fx");
    }
    ImGui::EndChild();
}

void GuiApp::draw_tab_bar() {
    if (ImGui::BeginTabBar("SessionTabs", ImGuiTabBarFlags_Reorderable)) {
        for (int i = 0; i < static_cast<int>(sessions_.size()); ++i) {
            bool open = true;
            const bool has_close_button = sessions_.size() > 1;
            if (ImGui::BeginTabItem(sessions_[i].label.c_str(), has_close_button ? &open : nullptr)) {
                active_session_ = i;
                ImGui::EndTabItem();
            }
            if (has_close_button && !open) pending_close_ = i;
        }
        if (ImGui::TabItemButton("+", ImGuiTabItemFlags_Trailing)) add_session();
        ImGui::EndTabBar();
    }
    if (pending_close_ >= 0) {
        close_session(pending_close_);
        pending_close_ = -1;
    }
}

void GuiApp::draw_preview_panel(Session& session, float width, float height) {
    ImGui::BeginChild("PreviewPanel", ImVec2(width, height), true,
                       ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::TextDisabled("Preview");
    ImGui::Separator();

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.x = std::max(avail.x, 1.0f);
    avail.y = std::max(avail.y, 1.0f);
    const ImVec2 p1 = ImVec2(p0.x + avail.x, p0.y + avail.y);
    const float scx = p0.x + avail.x * 0.5f;
    const float scy = p0.y + avail.y * 0.5f;

    Camera& cam = session.camera;
    cam.width = avail.x;
    cam.height = avail.y;
    if (!session.camera_initialized && session.last_road_count >= 0) {
        cam.fit(session.geometry.min_x, session.geometry.min_y, session.geometry.max_x, session.geometry.max_y);
        session.camera_initialized = true;
    }

    if (session.imagery.texture_id != 0 &&
        (session.applied_brightness != brightness_ || session.applied_contrast != contrast_)) {
        apply_brightness_contrast(session.imagery, brightness_, contrast_);
        session.applied_brightness = brightness_;
        session.applied_contrast = contrast_;
    }

    // Pan/zoom input first (so this frame's drag/wheel is already reflected in cam.cx/cy/scale
    // below, rather than lagging a frame behind).
    ImGui::InvisibleButton("##canvas", avail, ImGuiButtonFlags_MouseButtonLeft);
    const bool hovered = ImGui::IsItemHovered();
    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        const ImVec2 delta = ImGui::GetIO().MouseDelta;
        cam.cx -= delta.x / cam.scale;
        cam.cy += delta.y / cam.scale;
    }
    if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
        const float mx = ImGui::GetIO().MousePos.x, my = ImGui::GetIO().MousePos.y;
        const double wx = cam.cx + (mx - scx) / cam.scale;
        const double wy = cam.cy - (my - scy) / cam.scale;
        cam.scale = std::clamp(cam.scale * std::pow(1.1f, ImGui::GetIO().MouseWheel), 0.02f, 2000.0f);
        cam.cx = static_cast<float>(wx - (mx - scx) / cam.scale);
        cam.cy = static_cast<float>(wy + (my - scy) / cam.scale);
    }

    const auto world_to_screen = [&](double x, double y) {
        return ImVec2(scx + static_cast<float>(x - cam.cx) * cam.scale, scy - static_cast<float>(y - cam.cy) * cam.scale);
    };

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(p0, p1, IM_COL32(10, 10, 12, 255));
    draw_list->PushClipRect(p0, p1, true);

    if (show_imagery_ && session.imagery.texture_id != 0) {
        const ImVec2 tl = world_to_screen(session.imagery.min_x, session.imagery.max_y);
        const ImVec2 br = world_to_screen(session.imagery.max_x, session.imagery.min_y);
        draw_list->AddImage(ImTextureRef(static_cast<ImTextureID>(session.imagery.texture_id)), tl, br);
    }

    std::vector<ImVec2> screen_pts;
    if (show_center_ || show_lanes_) {
        for (const auto& road : session.geometry.roads) {
            if (show_center_ && road.centerline.size() >= 2) {
                screen_pts.clear();
                for (const auto& wp : road.centerline) screen_pts.push_back(world_to_screen(wp.x, wp.y));
                draw_list->AddPolyline(screen_pts.data(), static_cast<int>(screen_pts.size()), kColorCenter, 0, 2.0f);
            }
            if (show_lanes_) {
                for (const auto* edge : {&road.left_edge, &road.right_edge}) {
                    if (edge->size() < 2) continue;
                    screen_pts.clear();
                    for (const auto& wp : *edge) screen_pts.push_back(world_to_screen(wp.x, wp.y));
                    draw_list->AddPolyline(screen_pts.data(), static_cast<int>(screen_pts.size()), kColorLane, 0, 1.0f);
                }
            }
        }
    }
    if (show_markings_) {
        for (const auto& marking : session.geometry.lane_markings) {
            for (const auto& [start, end] : dash_index_ranges(marking)) {
                screen_pts.clear();
                for (int i = start; i < end; ++i) screen_pts.push_back(world_to_screen(marking[i].x, marking[i].y));
                draw_list->AddPolyline(screen_pts.data(), static_cast<int>(screen_pts.size()), kColorMarking, 0, 1.0f);
            }
        }
    }

    draw_list->PopClipRect();

    if (session.last_road_count < 0) {
        const char* placeholder = "No conversion yet -- set Input/Output in Details and press Convert.";
        const ImVec2 text_size = ImGui::CalcTextSize(placeholder);
        draw_list->AddText(ImVec2(p0.x + (avail.x - text_size.x) * 0.5f, p0.y + (avail.y - text_size.y) * 0.5f),
                            IM_COL32(160, 160, 160, 255), placeholder);
    } else if (session.imagery.texture_id == 0 && !session.imagery_status.empty()) {
        draw_list->AddText(ImVec2(p0.x + 6, p0.y + 6), IM_COL32(220, 160, 90, 255), session.imagery_status.c_str());
    }

    ImGui::EndChild();
}

void GuiApp::draw_details_panel(Session& session, float width, float height) {
    ImGui::BeginChild("DetailsPanel", ImVec2(width, height), true);
    ImGui::TextDisabled("Details");
    ImGui::Separator();

    Options& o = session.options;

    if (ImGui::CollapsingHeader("Files", ImGuiTreeNodeFlags_DefaultOpen)) {
        input_text("Input .osm/.pbf", o.input);
        input_text("Output .xodr", o.output);
        input_text("Header name", o.name);
        input_text("Report path (optional)", o.report_path);
    }

    if (ImGui::CollapsingHeader("Projection origin")) {
        ImGui::Checkbox("Override origin (default: first node in the input)", &session.use_custom_origin);
        if (session.use_custom_origin) {
            double lat = o.origin_lat.value_or(0.0);
            double lon = o.origin_lon.value_or(0.0);
            ImGui::InputDouble("Origin latitude", &lat, 0.0, 0.0, "%.7f");
            ImGui::InputDouble("Origin longitude", &lon, 0.0, 0.0, "%.7f");
            o.origin_lat = lat;
            o.origin_lon = lon;
        } else {
            o.origin_lat.reset();
            o.origin_lon.reset();
        }
    }

    if (ImGui::CollapsingHeader("Lanes & geometry")) {
        ImGui::InputDouble("Default lane width (m)", &o.default_lane_width, 0.0, 0.0, "%.2f");
        ImGui::InputDouble("Sidewalk width (m)", &o.sidewalk_width, 0.0, 0.0, "%.2f");
        ImGui::Checkbox("Left-hand traffic", &o.left_hand_traffic);
        ImGui::Checkbox("Curve fit", &o.curve_fit);
        ImGui::Checkbox("Fix link continuity", &o.fix_link_continuity);
    }

    if (ImGui::CollapsingHeader("Junctions")) {
        ImGui::InputInt("Junction degree", &o.junction_degree);
        ImGui::InputDouble("Turn radius (m)", &o.junction_turn_radius, 0.0, 0.0, "%.2f");
        ImGui::Checkbox("Adaptive turn radius", &o.adaptive_turn_radius);
        ImGui::Checkbox("Merge junctions", &o.merge_junctions);
        ImGui::InputDouble("Cluster max gap (m)", &o.junction_cluster_max_gap, 0.0, 0.0, "%.2f");
        ImGui::Checkbox("Absorb signal setbacks", &o.absorb_signal_setbacks);
        ImGui::InputDouble("Signal setback max gap (m)", &o.junction_signal_setback_max_gap, 0.0, 0.0, "%.2f");
        ImGui::InputDouble("Signal search radius (m)", &o.signal_search_radius, 0.0, 0.0, "%.2f");
    }

    if (ImGui::CollapsingHeader("Merging & tapering")) {
        ImGui::Checkbox("Merge roads", &o.merge_roads);
        ImGui::Checkbox("Adaptive lane taper", &o.adaptive_lane_taper);
        ImGui::InputDouble("Lane taper length (m)", &o.lane_taper_length, 0.0, 0.0, "%.2f");
        ImGui::Checkbox("Bridge lane count changes", &o.bridge_lane_count_changes);
    }

    ImGui::Separator();
    ImGui::TextWrapped("%s", session.status.empty() ? "Not converted yet." : session.status.c_str());
    if (!session.log.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("Warnings:");
        ImGui::BeginChild("Log", ImVec2(0, 120), true);
        ImGui::TextUnformatted(session.log.c_str());
        ImGui::EndChild();
    }

    ImGui::EndChild();
}

void GuiApp::draw_status_bar() {
    ImGui::Separator();
    if (!sessions_.empty()) {
        const Session& session = sessions_[active_session_];
        ImGui::TextUnformatted(session.status.empty() ? "Ready." : session.status.c_str());
    } else {
        ImGui::TextUnformatted("Ready.");
    }
}

void GuiApp::draw(float window_width, float window_height) {
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(window_width, window_height));
    constexpr ImGuiWindowFlags kFlags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_MenuBar;
    ImGui::Begin("osm2xodr", nullptr, kFlags);

    draw_command_bar();
    draw_toolbar();
    draw_tab_bar();

    if (!sessions_.empty()) {
        Session& session = sessions_[active_session_];

        const float status_bar_height = ImGui::GetTextLineHeightWithSpacing() + 8.0f;
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float main_height = std::max(avail.y - status_bar_height, 1.0f);
        const float total_width = std::max(avail.x, 1.0f);
        const float preview_width = std::floor(total_width * split_ratio_);
        const float details_width = std::max(total_width - preview_width - kSplitterWidth, 1.0f);

        draw_preview_panel(session, preview_width, main_height);

        ImGui::SameLine();
        ImGui::Button("##splitter", ImVec2(kSplitterWidth, main_height));
        if (ImGui::IsItemActive())
            split_ratio_ = std::clamp(split_ratio_ + ImGui::GetIO().MouseDelta.x / total_width, 0.15f, 0.85f);
        if (ImGui::IsItemHovered() || ImGui::IsItemActive())
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);

        ImGui::SameLine();
        draw_details_panel(session, details_width, main_height);
    }

    draw_status_bar();
    ImGui::End();
}

} // namespace osm2xodr::gui
