#pragma once

#include "lgl_imagery.hpp"
#include "road_geometry.hpp"
#include "osm2xodr/options.hpp"

#include <string>
#include <vector>

namespace osm2xodr::gui {

// Preview canvas camera: world (osm2xodr local meters, +y = north) <-> screen pixel transform,
// one per session so switching tabs keeps each tab's own pan/zoom.
struct Camera {
    float width = 1.0f, height = 1.0f;
    float cx = 0.0f, cy = 0.0f; // world point mapped to the canvas center
    float scale = 1.0f;         // screen pixels per meter

    void fit(double min_x, double min_y, double max_x, double max_y, float margin = 1.15f);
};

// Adjustable appearance of one drawn line kind (reference line, road border, or lane markings) --
// color is RGBA in ImGui's usual 0..1 float form (ready for ColorEdit4), width in screen pixels.
struct LineStyle {
    float color[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float width = 1.0f;
};

// One independent "conversion session" -- the unit the tab bar switches between. Each tab owns
// a plain osm2xodr::Options, the exact struct the CLI populates from argv, so the details panel
// can bind widgets straight to it instead of keeping a separate GUI-side schema in sync.
struct Session {
    std::string label = "New Conversion";
    Options options;
    bool use_custom_origin = false; // options.origin_lat/lon are optional; drives that checkbox
    std::string ignore_highways_custom; // free-text extra highway=* values to ignore, comma-separated
    std::string status;             // last convert result/error for this session
    std::string log;                // accumulated warnings from the last run
    int last_road_count = -1;
    int last_junction_count = -1;

    // Preview state, populated by run_conversion() on success.
    PreviewGeometry geometry;
    double origin_lat = 0.0, origin_lon = 0.0; // the model's actual projector origin (not
    bool has_origin = false;                   // necessarily the same as options.origin_lat/lon,
                                                // which may have been left unset and auto-derived)
    ImageryTile imagery;
    std::string imagery_status;
    ImageryTile landuse; // LGL's official ATKIS Basis-DLM Landnutzung polygons, semi-transparent
    std::string landuse_status;
    Camera camera;
    bool camera_initialized = false;
    // What's currently baked into each tile's texture -- compared against GuiApp's global
    // brightness_/contrast_ (and landuse_brightness_/landuse_contrast_) each frame so a fresh
    // fetch (always uploaded unadjusted, modulo landuse's fixed base alpha) or a change to those
    // global sliders gets lazily re-applied whenever this session is the one being drawn.
    int applied_brightness = 0;
    float applied_contrast = 1.0f;
    int applied_landuse_brightness = 0;
    float applied_landuse_contrast = 1.0f;
};

// Owns all GUI state and draws the window chrome every frame: a command bar (menu), a toolbar,
// a tab bar switching between Sessions, a left/right split (preview | details) for the active
// session, and a status bar. Deliberately has no SDL/OpenGL/ImGui-backend code in it -- main.cpp
// owns the platform window and calls draw() once per frame between ImGui::NewFrame()/Render().
class GuiApp {
public:
    GuiApp();
    ~GuiApp();

    void draw(float window_width, float window_height);

    bool should_quit() const { return quit_requested_; }

private:
    void draw_command_bar();
    void draw_toolbar();
    void draw_tab_bar();
    void draw_preview_panel(Session& session, float width, float height);
    void draw_details_panel(Session& session, float width, float height);
    void draw_status_bar();
    void draw_line_style_dialog();

    void add_session();
    void close_session(int index);
    void run_conversion(Session& session);
    void fetch_imagery(Session& session);

    // Persists/restores view preferences only (layer toggles, brightness/contrast, line
    // styles) -- not sessions/Options, which are per-conversion working state, not settings.
    // See gui_app.cpp for the file location and format.
    void load_settings();
    void save_settings() const;

    std::vector<Session> sessions_;
    int active_session_ = 0;
    int pending_close_ = -1;
    int session_counter_ = 0;
    float split_ratio_ = 0.6f; // preview panel's share of the main area's width
    bool show_about_ = false;
    bool show_line_style_dialog_ = false;
    bool quit_requested_ = false;

    // Reference line = yellow (was red); road border = red (was yellow); markings = near-white
    // dashed. Adjustable via View > Line & Color Settings... (draw_line_style_dialog()).
    LineStyle reference_line_style_{{1.0f, 0.831f, 0.0f, 0.902f}, 2.0f};
    LineStyle road_border_style_{{1.0f, 0.314f, 0.314f, 0.784f}, 1.0f};
    LineStyle marking_style_{{0.922f, 0.922f, 0.922f, 0.863f}, 1.0f};
    // Rail/tram track lane edges (road.is_railway) draw with this instead of road_border_style_.
    LineStyle rail_track_style_{{0.55f, 0.68f, 0.75f, 0.9f}, 1.5f};
    // Lane markings on junction/intersection connector roads use their own (default smaller) dash
    // length/gap instead of the fixed kMarkingDashM/kMarkingGapM plain-road pattern -- connectors
    // are short, tightly curved segments where a 3m dash reads as one long chunky segment.
    float junction_marking_dash_m_ = 1.0f;
    float junction_marking_gap_m_ = 1.0f;
    // Stop lines (RoadSignal type "stop_line", one per traffic_light) -- solid, not dashed.
    LineStyle stop_line_style_{{1.0f, 0.96f, 0.85f, 0.95f}, 3.0f};

    // Layer visibility and imagery brightness/contrast are view preferences, not per-conversion
    // data -- global across tabs (unlike Options/geometry/camera, which are per-Session) so
    // switching tabs doesn't reset how you're looking at things.
    bool show_imagery_ = true;
    bool show_center_ = true;
    bool show_lanes_ = true;
    bool show_markings_ = false;
    bool show_stop_lines_ = true;
    bool show_landuse_ = false;
    int brightness_ = 0;
    float contrast_ = 1.0f;
    int landuse_brightness_ = 0;
    float landuse_contrast_ = 1.0f;
};

} // namespace osm2xodr::gui
