// Copyright (c) 2023 - Barton Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include <string>
#include "Scene.h"
#include "e4math.h"

class ProbingScene : public Scene {
private:
    enum ProbeMode {
        ZSurface,
        XNegative,
        XPositive,
        YNegative,
        YPositive,
        XCenter,
        YCenter,
        BoreCenter,
        RectangleCenter,
        ProbeModeCount,
    };

    enum ProbePhase {
        NoProbe,
        ZNegativePhase,
        XNegativePhase,
        XPositivePhase,
        YNegativePhase,
        YPositivePhase,
    };

    static constexpr uint32_t PROBE_TIMEOUT_MS = 90000;

    int        _selection      = 0;
    int        _mode           = ZSurface;
    e4_t       _plate          = e4_from_int(0);
    e4_t       _diameter       = e4_from_int(4);
    e4_t       _travel         = e4_from_int(10);
    int        _rate           = 50;
    e4_t       _retract        = e4_from_int(3);
    bool       _running        = false;
    bool       _held           = false;
    bool       _probe_pending  = false;
    bool       _probe_success  = false;
    size_t     _probe_n_axis   = 0;
    pos_t      _probe_axes[6]  = { 0 };
    pos_t      _first_contact  = 0;
    ProbePhase _phase          = NoProbe;
    uint32_t   _probe_deadline = 0;
    uint32_t   _last_error_expire = 0;
    const char* _result        = nullptr;
    bool        _result_ok     = false;
    bool        _relative_mode_active = false;

    static const char* modeName(int mode) {
        static const char* names[ProbeModeCount] = {
            "Z Surface", "X- Edge", "X+ Edge", "Y- Edge", "Y+ Edge",
            "X Center", "Y Center", "Bore Center", "Rect Center",
        };
        return names[mode];
    }

    bool edgeMode() const {
        return _mode == XNegative || _mode == XPositive || _mode == YNegative || _mode == YPositive;
    }

    bool centerMode() const {
        return _mode == XCenter || _mode == YCenter || _mode == BoreCenter || _mode == RectangleCenter;
    }

    bool twoAxisCenterMode() const { return _mode == BoreCenter || _mode == RectangleCenter; }

    int phaseAxis() const {
        return (_phase == XNegativePhase || _phase == XPositivePhase) ? 0
             : (_phase == YNegativePhase || _phase == YPositivePhase) ? 1
             : 2;
    }

    bool phaseIsNegative() const {
        return _phase == ZNegativePhase || _phase == XNegativePhase || _phase == YNegativePhase;
    }

    int totalPoints() const { return twoAxisCenterMode() ? 4 : (centerMode() ? 2 : 1); }

    int pointNumber() const {
        switch (_phase) {
            case XNegativePhase: return 1;
            case XPositivePhase: return 2;
            case YNegativePhase: return twoAxisCenterMode() ? 3 : 1;
            case YPositivePhase: return twoAxisCenterMode() ? 4 : 2;
            default: return 1;
        }
    }

    void setResult(const char* result) {
        _result = result;
        request_redisplay();
    }

    void restoreAbsoluteMode() {
        if (_relative_mode_active) {
            send_line("G90");
            _relative_mode_active = false;
        }
    }

    void finish(const char* result, bool success = false, bool restore_positioning = true) {
        bool controller_accepts_gcode = state == Idle || state == Cycle || state == Hold || state == DoorClosed;
        if (restore_positioning && controller_accepts_gcode) {
            restoreAbsoluteMode();
        }
        _running        = false;
        _held           = false;
        _probe_pending  = false;
        _phase          = NoProbe;
        _probe_deadline = 0;
        _result_ok      = success;
        setResult(result);
    }

    void finishAlarm() {
        switch (lastAlarm) {
            case 4:
                finish("Probe was active at start", false, false);
                break;
            case 5:
                finish("No contact within travel", false, false);
                break;
            default:
                finish("Probe stopped by alarm", false, false);
                break;
        }
    }

    void sendProbe(int axis, bool negative, e4_t distance) {
        e4_t signed_distance = negative ? -distance : distance;
        _relative_mode_active = true;
        send_linef("G21 G91 G38.2 F%d %c%s", _rate, axisNumToChar(axis), e4_to_cstr(signed_distance, 3));
        _probe_deadline = milliseconds() + PROBE_TIMEOUT_MS;
    }

    void sendRetract(int axis, bool positive) {
        e4_t distance = positive ? _retract : -_retract;
        send_linef("G21 G91 G0 %c%s", axisNumToChar(axis), e4_to_cstr(distance, 3));
    }

    void startProbe() {
        if (state != Idle || _running || _rate <= 0 || _travel <= 0 || _retract <= 0 || _diameter <= 0) {
            return;
        }

        _running = true;
        _held    = false;
        _result  = nullptr;
        _last_error_expire = errorExpire;

        switch (_mode) {
            case ZSurface:
                _phase = ZNegativePhase;
                break;
            case XNegative:
            case XCenter:
            case BoreCenter:
            case RectangleCenter:
                _phase = XNegativePhase;
                break;
            case XPositive:
                _phase = XPositivePhase;
                break;
            case YNegative:
            case YCenter:
                _phase = YNegativePhase;
                break;
            case YPositive:
                _phase = YPositivePhase;
                break;
            default:
                finish("Invalid probe mode");
                return;
        }

        sendProbe(phaseAxis(), phaseIsNegative(), _travel);
        reDisplay();
    }

    void finishEdge(int axis, bool negative) {
        e4_t radius     = e4_scale(_diameter, 1, 2);
        e4_t coordinate = negative ? radius : -radius;
        send_linef("G10 L20 P0 %c%s", axisNumToChar(axis), e4_to_cstr(coordinate, 3));
        sendRetract(axis, negative);
        restoreAbsoluteMode();
        finish("Edge captured + retracted", true);
    }

    void finishCenterAxis(int axis, pos_t second_contact) {
        pos_t midpoint = (pos_t)(((int64_t)_first_contact + second_contact) / 2);

        // Clear the second face before moving to the measured machine-coordinate midpoint.
        sendRetract(axis, false);
        send_linef("G90 G53 G0 %c%s", axisNumToChar(axis), e4_to_cstr(midpoint, 4));
        send_linef("G10 L20 P0 %c0", axisNumToChar(axis));

        if (axis == 0 && twoAxisCenterMode()) {
            _phase = YNegativePhase;
            sendProbe(1, true, _travel);
            request_redisplay();
            return;
        }

        restoreAbsoluteMode();
        finish(twoAxisCenterMode() ? "X/Y center set" : "Axis center set", true);
    }

    void processProbeReport() {
        _probe_pending = false;
        if (!_running) {
            return;
        }
        if (!_probe_success || _probe_n_axis <= (size_t)phaseAxis()) {
            // G38.2 raises an alarm after an unsuccessful report. Restore G90
            // only after $X has unlocked the controller.
            finish("Probe did not contact", false, false);
            return;
        }

        int axis = phaseAxis();
        if (_mode == ZSurface) {
            send_linef("G10 L20 P0 Z%s", e4_to_cstr(_plate, 3));
            sendRetract(2, true);
            restoreAbsoluteMode();
            finish("Z set + retracted", true);
            return;
        }

        if (edgeMode()) {
            finishEdge(axis, phaseIsNegative());
            return;
        }

        if (phaseIsNegative()) {
            _first_contact = _probe_axes[axis];
            sendRetract(axis, true);
            _phase = axis == 0 ? XPositivePhase : YPositivePhase;
            sendProbe(axis, false, e4_scale(_travel, 2, 1));
            request_redisplay();
            return;
        }

        finishCenterAxis(axis, _probe_axes[axis]);
    }

    void adjustE4(e4_t& value, int delta, e4_t minimum, e4_t maximum, const char* preference) {
        value += delta * 1000;  // 0.1 mm per scaled encoder step
        if (value < minimum) value = minimum;
        if (value > maximum) value = maximum;
        setPref(preference, value);
    }

public:
    ProbingScene() : Scene("Probe", 4) {}

    void onDialButtonPress() override {
        if (!_running) {
            pop_scene();
        }
    }

    void onLeftFlick() override {
        if (!_running) {
            pop_scene();
        }
    }

    void onGreenButtonPress() override {
        if (_running) {
            if (state == Cycle) {
                fnc_realtime(FeedHold);
                _held = true;
                _probe_deadline = 0;
            } else if (state == Hold || state == DoorClosed) {
                fnc_realtime(CycleStart);
                _held = false;
                _probe_deadline = milliseconds() + PROBE_TIMEOUT_MS;
            }
        } else if (state == Idle) {
            startProbe();
        } else if (state == Alarm) {
            send_line("$X");
            restoreAbsoluteMode();
        }
    }

    void onRedButtonPress() override {
        if (_running || state == Cycle || state == Hold || state == DoorClosed) {
            fnc_realtime(Reset);
            // Soft reset restores the controller's modal defaults. Do not
            // append a G-code command to this realtime reset path.
            _relative_mode_active = false;
            finish("Probe aborted", false, false);
        }
    }

    void onTouchClick() override {
        if (_running || state != Idle) {
            return;
        }
        if (touchY >= 60 && touchY < 195) {
            _selection = (touchY - 60) / 27;
            if (_selection > 4) _selection = 4;
        } else {
            rotateNumberLoop(_selection, 1, 0, 4);
        }
        reDisplay();
        ackBeep();
    }

    void onEncoder(int delta) override {
        if (_running || state != Idle || delta == 0) {
            return;
        }

        switch (_selection) {
            case 0:
                rotateNumberLoop(_mode, delta, 0, (int)ProbeModeCount - 1);
                setPref("Mode", _mode);
                break;
            case 1:
                if (_mode == ZSurface) {
                    adjustE4(_plate, delta, e4_from_int(0), e4_from_int(100), "Offset");
                } else {
                    adjustE4(_diameter, delta, 1000, e4_from_int(100), "Diameter");
                }
                break;
            case 2:
                adjustE4(_travel, delta, 1000, e4_from_int(1000), "Travel4");
                break;
            case 3:
                _rate += delta * 10;
                if (_rate < 1) _rate = 1;
                if (_rate > 10000) _rate = 10000;
                setPref("Rate", _rate);
                break;
            case 4:
                adjustE4(_retract, delta, 1000, e4_from_int(100), "Retract4");
                break;
        }
        _result = nullptr;
        reDisplay();
    }

    void onProbe(const pos_t* axes, bool success, size_t n_axis) override {
        if (!_running) {
            return;
        }
        _probe_success = success;
        _probe_n_axis  = n_axis > 6 ? 6 : n_axis;
        for (size_t axis = 0; axis < _probe_n_axis; ++axis) {
            _probe_axes[axis] = axes[axis];
        }
        _probe_pending = true;
    }

    void onPoll() override {
        if (_probe_pending) {
            processProbeReport();
        }
        if (_running && errorExpire != _last_error_expire) {
            _last_error_expire = errorExpire;
            finish("Probe command rejected");
            return;
        }
        if (_running && !_held && _probe_deadline && (int32_t)(milliseconds() - _probe_deadline) >= 0) {
            finish("Probe response timed out");
        }
    }

    void onStateChange(state_t old_state) override {
        if (_running && (state == Alarm || state == Disconnected || state == Critical)) {
            if (state == Alarm) {
                finishAlarm();
            } else {
                finish("Probe interrupted");
            }
        } else {
            if (_running && (state == Hold || state == DoorClosed)) {
                _held           = true;
                _probe_deadline = 0;
            } else if (_running && state == Cycle && (old_state == Hold || old_state == DoorClosed)) {
                _held           = false;
                _probe_deadline = milliseconds() + PROBE_TIMEOUT_MS;
            }
            request_redisplay();
        }
    }

    void onDROChange() override { request_redisplay(); }

    void onEntry(void* arg) override {
        if (initPrefs()) {
            static_assert(sizeof(e4_t) == sizeof(int));
            getPref("Mode", &_mode);
            getPref("Offset", reinterpret_cast<int*>(&_plate));
            getPref("Diameter", reinterpret_cast<int*>(&_diameter));
            getPref("Travel4", reinterpret_cast<int*>(&_travel));
            getPref("Rate", &_rate);
            getPref("Retract4", reinterpret_cast<int*>(&_retract));
        }

        if (_mode < 0 || _mode >= ProbeModeCount) _mode = ZSurface;
        if (_plate < 0) _plate = 0;
        if (_diameter <= 0) _diameter = e4_from_int(4);
        if (_travel <= 0) _travel = e4_from_int(50);
        if (_rate <= 0) _rate = 100;
        if (_retract <= 0) _retract = e4_from_int(3);
        _result = nullptr;
    }

    void reDisplay() override {
        background();
        drawMenuTitle(current_scene->name());
        drawStatus();

        const char* green = "";
        const char* red   = "";

        if (_running) {
            centered_text(modeName(_mode), 69, LIGHTGREY, SMALL);

            char progress[32];
            if (_phase == ZNegativePhase) {
                snprintf(progress, sizeof(progress), "Probing Z-");
            } else {
                snprintf(progress, sizeof(progress), "Point %d/%d: %c%c",
                         pointNumber(), totalPoints(), axisNumToChar(phaseAxis()), phaseIsNegative() ? '-' : '+');
            }
            centered_text(progress, 94, _held ? YELLOW : GREEN, SMALL);

            DRO dro(18, 113, display_short_side() - 36, 24, SMALL, 26);
            dro.draw(0, phaseAxis() == 0);
            dro.draw(1, phaseAxis() == 1);
            dro.draw(2, phaseAxis() == 2);

            LED led(display_short_side() / 2, 199, 8, 2);
            led.draw(myProbeSwitch);

            red   = "Abort";
            green = _held ? "Resume" : "Hold";
        } else if (state == Idle) {
            int width = display_short_side() - 48;
            Stripe button(24, 60, width, 24, TINY, 27);
            button.draw("Mode", modeName(_mode), _selection == 0);
            button.draw(_mode == ZSurface ? "Plate mm" : "Probe Dia", e4_to_cstr(_mode == ZSurface ? _plate : _diameter, 2), _selection == 1);
            button.draw("Travel mm", e4_to_cstr(_travel, 1), _selection == 2);
            button.draw("Feed mm/m", intToCStr(_rate), _selection == 3);
            button.draw("Retract mm", e4_to_cstr(_retract, 1), _selection == 4);

            green = "Probe";
        } else if (state == Alarm) {
            if (lastAlarm == 5) {
                centered_text("No probe contact", 95, YELLOW, SMALL);
                centered_text("Travel limit reached", 125, WHITE, TINY);
                centered_text("Increase travel", 150, LIGHTGREY, TINY);
                centered_text("or reposition", 170, LIGHTGREY, TINY);
            } else if (lastAlarm == 4) {
                centered_text("Check probe", 95, YELLOW, SMALL);
            } else {
                centered_text("Probing aborted", 100, WHITE, SMALL);
            }
            green = "Unlock";
        } else {
            centered_text("Wait for Idle", 115, WHITE, SMALL);
        }

        if (_result && !_running && state == Idle) {
            centered_text(_result, 204, _result_ok ? GREEN : YELLOW, TINY);
        }

        drawButtonLegends(red, green, _running ? "" : "Back");
        drawError();
        refreshDisplay();
    }
};

ProbingScene probingScene;
