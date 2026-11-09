// furnace_dashboard.cpp
//
// An interactive, SFML-driven control-dashboard for the furnace PID regulator.
//
// Why this exists (and why it matters on a resume): real industrial panels are
// *interactive* — an engineer tweaks Kp/Ki/Kd and a setpoint while the loop
// runs, and watches the temperature respond in real time. This demo reproduces
// exactly that:
//
//   * live sliders retune the PID on the fly (gain changes take effect next
//     step, so you immediately see the effect on overshoot/settling);
//   * icon buttons Run/Pause/Stop/Reset drive the simulation;
//   * a thermometer gauge + valve bar + live history plot show the response;
//   * an over-temperature alarm lamp (phosphor "warning" icon, reused from the
//     3D GAME ENGINE) lights red when the loop is driven into an unsafe region.
//
// Assets reused from the 3D GAME ENGINE (see README attribution):
//   * phosphor-icons  -> examples/assets/icons/icon_*.png (white-on-transparent,
//                        tinted at draw time, so one 64x64 sheet colours every
//                        context: accent / setpoint / alarm / muted).
//   * JetBrainsMono   -> examples/assets/fonts/...ttf (monospaced control font).
//
// Build (CMake auto-detects SFML; needs a display to run the window):
//   cmake -S . -B build && cmake --build build && ./build/furnace_dashboard
//   # headless preview:  xvfb-run -a ./build/furnace_dashboard

#include "pid/PIDController.hpp"

#include <SFML/Graphics.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Layout (pixel space; default view so mouse coords are 1:1)
// ---------------------------------------------------------------------------
namespace lay {
    constexpr int WIN_W = 1200, WIN_H = 760;

    // Left region: live furnace view.
    constexpr float G_LEFT_X0 =  50, G_LEFT_X1 = 190;          // thermometer rail
    constexpr float G_TOP_Y0  = 140, G_TOP_Y1  = 580;          // 0..T_MAX scale
    constexpr float VALVE_Y   = 108, VALVE_X0 = 210, VALVE_X1 = 660;
    constexpr float PLOT_X0   = 210, PLOT_X1   = 660;
    constexpr float PLOT_Y0   = 170, PLOT_Y1   = 580;
    constexpr float PLOT_W    = PLOT_X1 - PLOT_X0;
    constexpr float PLOT_H    = PLOT_Y1 - PLOT_Y0;

    // Right region: control panel.
    constexpr float PANEL_X0  = 720, PANEL_X1 = 1170;
    constexpr float PANEL_TITLE_Y = 120;
    constexpr float SLIDER_X0 = 790;            // slider label/icon column
    constexpr float TRACK_X0  = 850;            // slider track start
    constexpr float SLIDER_Y0 = 176;
    constexpr float ROW_H     = 52;
    constexpr float BTN_Y     = 432;
    constexpr float STATUS_Y  = 562;

    constexpr float SIM_DT      = 0.10f;   // model step (s)
    constexpr float STEP_WALL   = 0.025f;  // wall-clock seconds per model step (4x speedup)
    constexpr int   SIM_STEPS   = 400;     // simulated seconds: 40
}

// ---------------------------------------------------------------------------
// Colour palette — dark industrial with accent / alarm colours
// ---------------------------------------------------------------------------
namespace col {
    const sf::Color BACK         = sf::Color(18, 20, 26);
    const sf::Color PANEL        = sf::Color(30, 32, 42);
    const sf::Color RAIL         = sf::Color(52, 58, 70);
    const sf::Color TEXT         = sf::Color(222, 226, 236);
    const sf::Color TEXT_DIM     = sf::Color(148, 154, 168);
    const sf::Color BLUE         = sf::Color(70, 170, 255);   // tuning accent
    const sf::Color AMBER        = sf::Color(255, 176, 56);   // setpoint
    const sf::Color GREEN        = sf::Color(76, 200, 110);   // running
    const sf::Color RED          = sf::Color(236, 90, 80);    // fault
    const sf::Color KNOB         = sf::Color(228, 232, 242);
    const sf::Color PANEL_EDGE   = sf::Color(60, 66, 80);
    const sf::Color GRID         = sf::Color(70, 76, 88, 120);
}

// ---------------------------------------------------------------------------
// Coordinate mapping (temperature/pixel helpers)
// ---------------------------------------------------------------------------
namespace util {
    inline float thermoY(float t) {
        return lay::G_TOP_Y1 - (t / 2000.0f) * (lay::G_TOP_Y1 - lay::G_TOP_Y0);
    }
    inline float plotY(float t) {
        return lay::PLOT_Y0 + (1.0f - t / 2000.0f) * lay::PLOT_H;
    }
    inline float plotX(int i, int n) {
        return lay::PLOT_X0 + (static_cast<float>(i) / (n - 1)) * lay::PLOT_W;
    }
}

// ---------------------------------------------------------------------------
// Icon: a tintable phosphor glyph texture
// ---------------------------------------------------------------------------
struct Icon {
    sf::Texture tex;
    bool loaded = false;
    bool load(const std::string& path) {
        loaded = tex.loadFromFile(path);
        if (loaded) tex.setSmooth(true);
        return loaded;
    }
    // White-on-transparent source * tint => tinted glyph where ink is, transparent elsewhere.
    void draw(sf::RenderWindow& w, sf::Vector2f pos, sf::Color tint,
              float scale = 0.40f) const {
        if (!loaded) return;
        sf::Sprite s(tex);
        s.setPosition(pos);
        s.setScale(scale, scale);
        s.setColor(tint);
        w.draw(s);
    }
};

struct IconSet {
    Icon play, pause, stop, reset, settings, temp, heat, setpoint, alarm, drop;
    void loadAll(const std::string& dir) {
        play.load(dir + "icon_play.png");
        pause.load(dir + "icon_pause.png");
        stop.load(dir + "icon_stop.png");
        reset.load(dir + "icon_reset.png");
        settings.load(dir + "icon_settings.png");
        temp.load(dir + "icon_temp.png");
        heat.load(dir + "icon_heat.png");
        setpoint.load(dir + "icon_setpoint.png");
        alarm.load(dir + "icon_alarm.png");
        drop.load(dir + "icon_drop.png");
    }
} icons;

// ---------------------------------------------------------------------------
// Slider: icon + label + track + knob + numeric value
// ---------------------------------------------------------------------------
struct Slider {
    const Icon* icon;
    std::string name;
    std::string fmt;          // e.g. "%.2f"
    sf::Vector2f iconPos;
    sf::Vector2f trackPos;
    float trackW = 200, trackH = 22;
    float minVal, maxVal;
    float norm = 0.5f;        // 0..1 normalized position
    sf::Color tint;
    bool dragging = false;
    bool hovered  = false;

    Slider(const Icon* ic, const std::string& n, const std::string& f,
           sf::Vector2f ip, sf::Vector2f tp, float lo, float hi, float def,
           sf::Color t)
        : icon(ic), name(n), fmt(f), iconPos(ip), trackPos(tp),
          minVal(lo), maxVal(hi), tint(t) {
        norm = std::clamp((def - lo) / (hi - lo), 0.0f, 1.0f);
    }

    float value() const { return minVal + norm * (maxVal - minVal); }

    bool handle(sf::Vector2f mouse, bool pressEdge) {
        hovered = sf::FloatRect(trackPos.x, trackPos.y, trackW, trackH).contains(mouse) ||
                  sf::FloatRect(iconPos.x, iconPos.y, 30, 24).contains(mouse);
        const bool down = sf::Mouse::isButtonPressed(sf::Mouse::Left);
        if (pressEdge && hovered) dragging = true;
        if (!down) dragging = false;
        if (dragging && down) {
            norm = std::clamp((mouse.x - trackPos.x) / trackW, 0.0f, 1.0f);
            return true;
        }
        return false;
    }

    void draw(sf::RenderWindow& w, sf::Font& font) const {
        icon->draw(w, iconPos, tint, 0.34f);

        sf::RectangleShape rail(sf::Vector2f(trackW, trackH));
        rail.setPosition(trackPos);
        rail.setFillColor(col::RAIL);
        w.draw(rail);

        const float kx = trackPos.x + norm * trackW;
        sf::RectangleShape fill(sf::Vector2f(std::max(0.0f, kx - trackPos.x), trackH));
        fill.setPosition(trackPos);
        fill.setFillColor(tint);
        w.draw(fill);

        sf::CircleShape knob(11);
        knob.setOrigin(11, 11);
        knob.setPosition(kx, trackPos.y + trackH / 2.0f);
        knob.setFillColor(dragging ? col::KNOB : sf::Color(210, 214, 224, 220));
        w.draw(knob);

        // Name + numeric value to the right of the track.
        sf::Text tname; tname.setFont(font); tname.setString(name);
        tname.setCharacterSize(15); tname.setFillColor(col::TEXT_DIM);
        tname.setPosition(trackPos.x + trackW + 12, trackPos.y + 2);
        w.draw(tname);

        char buf[32];
        std::snprintf(buf, sizeof(buf), fmt.c_str(), value());
        sf::Text tval; tval.setFont(font); tval.setString(buf);
        tval.setCharacterSize(15); tval.setFillColor(hovered ? col::BLUE : col::TEXT);
        tval.setPosition(trackPos.x + trackW + 12, trackPos.y + 20);
        w.draw(tval);
    }
};

// ---------------------------------------------------------------------------
// Button: a textured icon button with a label, hover/press feedback.
// ---------------------------------------------------------------------------
struct Button {
    const Icon* icon;
    std::string label;
    sf::Vector2f pos;
    float w = 116, h = 48;
    sf::Color bg, bgHover;
    sf::Color tint;
    int  id;
    bool hovered = false;

    Button(const Icon* ic, const std::string& lab, sf::Vector2f p,
           sf::Color b, sf::Color bh, sf::Color t, int i)
        : icon(ic), label(lab), pos(p), bg(b), bgHover(bh), tint(t), id(i) {}

    bool contains(sf::Vector2f m) const {
        return sf::FloatRect(pos.x, pos.y, w, h).contains(m);
    }
    bool handle(sf::Vector2f mouse, bool pressEdge) {
        hovered = contains(mouse);
        return hovered && pressEdge && sf::Mouse::isButtonPressed(sf::Mouse::Left);
    }
    void draw(sf::RenderWindow& window, sf::Font& font) const {
        sf::RectangleShape bgRec(sf::Vector2f(w, h));
        bgRec.setPosition(pos);
        bgRec.setFillColor(hovered ? bgHover : bg);
        bgRec.setOutlineThickness(1.0f);
        bgRec.setOutlineColor(col::PANEL_EDGE);
        window.draw(bgRec);

        icon->draw(window, sf::Vector2f(pos.x + 14, pos.y + 10), tint, 0.46f);

        sf::Text lab; lab.setFont(font); lab.setString(label);
        lab.setCharacterSize(15); lab.setFillColor(col::TEXT);
        lab.setPosition(pos.x + 48, pos.y + 16);
        window.draw(lab);
    }
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    using automation::PIDController;
    using namespace lay;          // bring layout constants into scope

    // --auto : run the regulation from start to finish without clicking
    // "RUN", then print a summary and exit. Useful for recording a demo
    // pass (e.g. under a virtual framebuffer) and for headless verification.
    const bool autoMode =
        (argc > 1) && (std::string(argv[1]) == "--auto" || std::string(argv[1]) == "-a");

    // --- Window + font ---
    sf::RenderWindow window(sf::VideoMode(WIN_W, WIN_H),
                            "Furnace PID — Industrial Control Dashboard  (Esc to quit)");
    window.setFramerateLimit(60);

    sf::Font font;
    const char* fontPaths[] = {
        "examples/assets/fonts/JetBrainsMono-SemiBold.ttf",
        "/home/run-time-terror/Documents/learn.me/automation-logic-and-control/examples/assets/fonts/JetBrainsMono-SemiBold.ttf",
    };
    bool fontOk = false;
    for (const char* p : fontPaths) if (font.loadFromFile(p)) { fontOk = true; break; }
    if (!fontOk) std::printf("Warning: JetBrainsMono not loaded; text will not render.\n");

    icons.loadAll("examples/assets/icons/");

    // --- Plant + controller ---
    constexpr float SETPOINT = 1600.0f;
    PIDController pid(0.6, 0.3, 0.2, SETPOINT, 0.0, 100.0);
    pid.setSampleTime(lay::SIM_DT);

    const float AMBIENT   = 0.0f;
    const float PLANT_K   = 4.0f;
    const float LOSS      = 0.20f;
    const float T_MAX     = 2000.0f;
    const float SAFE_MAX  = 2050.0f;          // beyond this => over-temperature fault

    float temperature = AMBIENT;
    float valve       = 0.0f;

    std::vector<float> history;
    history.reserve(lay::SIM_STEPS);

    // --- Control widgets ---
    Slider sKp (&icons.settings, "PROPORTIONAL", "%.2f",
                sf::Vector2f(lay::SLIDER_X0, lay::SLIDER_Y0 + 0 * lay::ROW_H),
                sf::Vector2f(lay::TRACK_X0,    lay::SLIDER_Y0 - 6 + 0 * lay::ROW_H),
                0.0f, 10.0f, 0.6f, col::BLUE);       // Kp default
    Slider sKi (&icons.heat,    "INTEGRAL",   "%.3f",
                sf::Vector2f(lay::SLIDER_X0, lay::SLIDER_Y0 + 1 * lay::ROW_H),
                sf::Vector2f(lay::TRACK_X0,  lay::SLIDER_Y0 - 6 + 1 * lay::ROW_H),
                0.0f, 0.5f,  0.3f, col::BLUE);       // Ki default
    Slider sKd (&icons.drop,    "DERIVATIVE", "%.2f",
                sf::Vector2f(lay::SLIDER_X0, lay::SLIDER_Y0 + 2 * lay::ROW_H),
                sf::Vector2f(lay::TRACK_X0,  lay::SLIDER_Y0 - 6 + 2 * lay::ROW_H),
                0.0f, 5.0f,  0.2f, col::BLUE);       // Kd default
    Slider sSp (&icons.setpoint,"SETPOINT",   "%.0f",
                sf::Vector2f(lay::SLIDER_X0, lay::SLIDER_Y0 + 3 * lay::ROW_H),
                sf::Vector2f(lay::TRACK_X0,  lay::SLIDER_Y0 - 6 + 3 * lay::ROW_H),
                1000.0f, 2000.0f, SETPOINT, col::AMBER);
    Slider sAmb(&icons.drop,    "AMBIENT",    "%.0f",
                sf::Vector2f(lay::SLIDER_X0, lay::SLIDER_Y0 + 4 * lay::ROW_H),
                sf::Vector2f(lay::TRACK_X0,  lay::SLIDER_Y0 - 6 + 4 * lay::ROW_H),
                0.0f, 300.0f, AMBIENT, col::BLUE);

    Button btnRun (&icons.play,   "RUN",  sf::Vector2f(lay::PANEL_X0 + 30, lay::BTN_Y),           col::PANEL, sf::Color(40,46,58), col::GREEN, 1);
    Button btnStop(&icons.stop,   "STOP", sf::Vector2f(lay::PANEL_X0 + 166, lay::BTN_Y),          col::PANEL, sf::Color(48,42,50), col::RED,    2);
    Button btnReset(&icons.reset,  "RESET", sf::Vector2f(lay::PANEL_X0 + 302, lay::BTN_Y),         col::PANEL, sf::Color(48,44,42), col::AMBER,  3);

    Slider* sliders[] = { &sKp, &sKi, &sKd, &sSp, &sAmb };

    // --- Runtime state ---
    bool running   = autoMode;   // --auto mode starts the regulation immediately
    bool faulted   = false;
    int  simStep   = 0;
    sf::Clock wall;
    sf::Clock stepClk;
    bool prevMouseDown = false;

    auto resetSim = [&]() {
        pid.reset();
        temperature = sAmb.value();
        valve = 0.0f;
        history.clear();
        simStep = 0;
        faulted = false;
        running = false;
        stepClk.restart();
    };

    while (window.isOpen()) {
        // ---- input ----
        sf::Event ev;
        while (window.pollEvent(ev)) {
            if (ev.type == sf::Event::Closed) window.close();
            if (ev.type == sf::Event::KeyPressed &&
                ev.key.code == sf::Keyboard::Escape) window.close();
        }
        const sf::Vector2f mouse =
            sf::Vector2f(sf::Mouse::getPosition(window));  // 1:1 with default view
        const bool mouseDown   = sf::Mouse::isButtonPressed(sf::Mouse::Left);
        const bool pressEdge    = mouseDown && !prevMouseDown;
        prevMouseDown = mouseDown;

        // ---- widget interaction ----
        for (auto* s : sliders) {
            if (faulted) {
                // sliders stay live for read-out / setup, but don't drive motion
            }
            s->handle(mouse, pressEdge);
        }
        // Push live tuning into the controller every frame.
        pid.setGains(sKp.value(), sKi.value(), sKd.value());
        pid.setSetpoint(sSp.value());

        if (btnRun.handle(mouse, pressEdge))  { running = !running; }
        if (btnStop.handle(mouse, pressEdge)) { running = false; faulted = false; }
        if (btnReset.handle(mouse, pressEdge)){ resetSim(); }

        // toggle Run/Pause icon + label
        btnRun.icon  = running ? &icons.pause : &icons.play;
        btnRun.label = running ? "PAUSE" : "RUN";

        // ---- simulation step ----
        if (running && !faulted && simStep < lay::SIM_STEPS) {
            if (stepClk.getElapsedTime().asSeconds() >= lay::STEP_WALL) {
                valve = pid.compute(temperature);
                temperature += PLANT_K * valve * lay::SIM_DT
                             - LOSS * (temperature - sAmb.value()) * lay::SIM_DT;
                if (temperature < sAmb.value()) temperature = sAmb.value();
                history.push_back(temperature);
                ++simStep;

                if (temperature > SAFE_MAX) {   // over-temperature trip
                    faulted = true;
                    running = false;
                }
                stepClk.restart();
            }
        }

        // ---- render ----
        window.clear(col::BACK);

        // Control-panel backdrop.
        sf::RectangleShape panel(sf::Vector2f(lay::PANEL_X1 - lay::PANEL_X0,
                                              lay::WIN_H - 2 * 60));
        panel.setPosition(lay::PANEL_X0, 60);
        panel.setFillColor(col::PANEL);
        window.draw(panel);

        sf::Text title; title.setFont(font); title.setString("CONTROL");
        title.setCharacterSize(20); title.setFillColor(col::TEXT_DIM);
        title.setPosition(lay::PANEL_TITLE_Y, 98);
        window.draw(title);

        for (auto* s : sliders) s->draw(window, font);
        btnRun.draw(window, font);
        btnStop.draw(window, font);
        btnReset.draw(window, font);

        // Status lamp: alarm icon tints red on fault, amber when paused, green when running.
        const bool done = (simStep >= lay::SIM_STEPS);
        sf::Color lamp = faulted ? col::RED : (running ? col::GREEN : col::AMBER);
        icons.alarm.draw(window, sf::Vector2f(lay::PANEL_X0 + 30, lay::STATUS_Y), lamp, 0.46f);
        char sbuf[96];
        std::snprintf(sbuf, sizeof(sbuf),
                      faulted ? "FAULT: OVER-TEMPERATURE"
                              : done   ? "COMPLETE — setpoint reached"
                                       : (running ? "RUNNING" : "PAUSED"));
        sf::Text status; status.setFont(font); status.setString(sbuf);
        status.setCharacterSize(16); status.setFillColor(lamp);
        status.setPosition(lay::PANEL_X0 + 80, lay::STATUS_Y + 12);
        window.draw(status);

        // --- Furnace thermometer gauge ---
        sf::RectangleShape railBg(sf::Vector2f(G_LEFT_X1 - G_LEFT_X0, G_TOP_Y1 - G_TOP_Y0));
        railBg.setPosition(G_LEFT_X0, G_TOP_Y0);
        railBg.setFillColor(sf::Color(40, 42, 50));
        window.draw(railBg);

        const float fillH = lay::G_TOP_Y1 - util::thermoY(temperature);
        sf::RectangleShape tempFill(sf::Vector2f(G_LEFT_X1 - G_LEFT_X0, fillH));
        tempFill.setPosition(G_LEFT_X0, G_TOP_Y1 - fillH);
        // colour ramp: blue -> amber -> red as it approaches setpoint/overheat
        sf::Color gaugeCol = temperature < SETPOINT
            ? sf::Color(60, 150, 210)
            : (temperature < T_MAX ? col::AMBER : col::RED);
        tempFill.setFillColor(gaugeCol);
        window.draw(tempFill);

        // setpoint marker
        const float spY = util::thermoY(SETPOINT);
        sf::Vertex spLine[] = {
            sf::Vertex(sf::Vector2f(G_LEFT_X0 - 8, spY), col::AMBER),
            sf::Vertex(sf::Vector2f(G_LEFT_X1 + 8, spY), col::AMBER),
        };
        window.draw(spLine, 2, sf::Lines);
        icons.temp.draw(window, sf::Vector2f(lay::G_LEFT_X0 - 34, spY - 10), col::AMBER, 0.36f);

        // Valve bar.
        sf::RectangleShape valveRail(sf::Vector2f(VALVE_X1 - VALVE_X0, 22));
        valveRail.setPosition(VALVE_X0, VALVE_Y);
        valveRail.setFillColor(col::RAIL);
        window.draw(valveRail);
        sf::RectangleShape valveFill(sf::Vector2f((valve / 100.0f) * (VALVE_X1 - VALVE_X0), 22));
        valveFill.setPosition(VALVE_X0, VALVE_Y);
        valveFill.setFillColor(col::GREEN);
        window.draw(valveFill);
        icons.heat.draw(window, sf::Vector2f(VALVE_X0 + 6, VALVE_Y + 2), col::RED, 0.38f);
        sf::Text vlabel; vlabel.setFont(font); vlabel.setString("VALVE %");
        vlabel.setCharacterSize(14); vlabel.setFillColor(col::TEXT_DIM);
        vlabel.setPosition(VALVE_X1 - 90, VALVE_Y - 2);
        window.draw(vlabel);

        // History plot: temperature curve + setpoint line.
        if (history.size() >= 2) {
            sf::VertexArray line(sf::LineStrip, history.size());
            for (size_t i = 0; i < history.size(); ++i) {
                line[i].position = sf::Vector2f(
                    util::plotX(static_cast<int>(i), static_cast<int>(history.size())),
                    util::plotY(history[i]));
                line[i].color = col::BLUE;
            }
            window.draw(line);
        }
        sf::Vertex spPlot[] = {
            sf::Vertex(sf::Vector2f(lay::PLOT_X0, util::plotY(SETPOINT)), col::AMBER),
            sf::Vertex(sf::Vector2f(lay::PLOT_X1, util::plotY(SETPOINT)), col::AMBER),
        };
        window.draw(spPlot, 2, sf::Lines);

        // Plot frame.
        sf::RectangleShape frame(sf::Vector2f(lay::PLOT_W, lay::PLOT_H));
        frame.setPosition(lay::PLOT_X0, lay::PLOT_Y0);
        frame.setFillColor(sf::Color(0, 0, 0, 0));
        frame.setOutlineThickness(1.0f);
        frame.setOutlineColor(col::PANEL_EDGE);
        window.draw(frame);

        window.display();

        // In --auto mode, run the whole regulation to steady state then exit.
        if (autoMode && simStep >= SIM_STEPS) {
            // Grab a frame for screenshots / README art (useful for an offline,
            // display-less build of the promotional materials). Uses the
            // non-deprecated Texture::update path (RenderWindow::capture is
            // deprecated in SFML 2.6).
            sf::Texture frame;
            frame.create(window.getSize().x, window.getSize().y);
            frame.update(window);
            frame.copyToImage().saveToFile("furnace_dashboard_screenshot.png");
            std::printf("[auto] final temperature = %.2f C, setpoint = %.0f C, "
                        "valve = %.2f %%, error = %.2f C\n",
                        temperature, SETPOINT, valve, std::abs(SETPOINT - temperature));
            std::printf("[auto] screenshot -> furnace_dashboard_screenshot.png\n");
            break;
        }
    }

    return 0;
}
