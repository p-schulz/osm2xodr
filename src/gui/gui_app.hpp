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

// One independent "conversion session" -- the unit the tab bar switches between. Each tab owns
// a plain osm2xodr::Options, the exact struct the CLI populates from argv, so the details panel
// can bind widgets straight to it instead of keeping a separate GUI-side schema in sync.
struct Session {
    std::string label = "New Conversion";
    Options options;
    bool use_custom_origin = false; // options.origin_lat/lon are optional; drives that checkbox
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
    Camera camera;
    bool camera_initialized = false;
    // What's currently baked into imagery's texture -- compared against GuiApp's global
    // brightness_/contrast_ each frame so a fresh fetch (always uploaded unadjusted) or a change
    // to those global sliders gets lazily re-applied whenever this session is the one being drawn.
    int applied_brightness = 0;
    float applied_contrast = 1.0f;
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

    void add_session();
    void close_session(int index);
    void run_conversion(Session& session);
    void fetch_imagery(Session& session);

    std::vector<Session> sessions_;
    int active_session_ = 0;
    int pending_close_ = -1;
    int session_counter_ = 0;
    float split_ratio_ = 0.6f; // preview panel's share of the main area's width
    bool show_about_ = false;
    bool quit_requested_ = false;

    // Layer visibility and imagery brightness/contrast are view preferences, not per-conversion
    // data -- global across tabs (unlike Options/geometry/camera, which are per-Session) so
    // switching tabs doesn't reset how you're looking at things.
    bool show_imagery_ = true;
    bool show_center_ = true;
    bool show_lanes_ = true;
    bool show_markings_ = false;
    int brightness_ = 0;
    float contrast_ = 1.0f;
};

} // namespace osm2xodr::gui
