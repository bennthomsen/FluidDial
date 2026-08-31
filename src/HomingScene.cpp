// Copyright (c) 2023 - Barton Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Scene.h"
#include "ConfigItem.h"

extern Scene statusScene;

#define HOMING_N_AXIS 6

static int homing_axis_count() {
    if (n_axes <= 0) {
        return 3;
    }
    return n_axes < HOMING_N_AXIS ? n_axes : HOMING_N_AXIS;
}

IntConfigItem homing_cycles[HOMING_N_AXIS] = {
    { "$/axes/x/homing/cycle" },
    { "$/axes/y/homing/cycle" },
    { "$/axes/z/homing/cycle" },
    { "$/axes/a/homing/cycle" },
    { "$/axes/b/homing/cycle" },
    { "$/axes/c/homing/cycle" },
};
BoolConfigItem homing_allows[HOMING_N_AXIS] = {
    { "$/axes/x/homing/allow_single_axis" },
    { "$/axes/y/homing/allow_single_axis" },
    { "$/axes/z/homing/allow_single_axis" },
    { "$/axes/a/homing/allow_single_axis" },
    { "$/axes/b/homing/allow_single_axis" },
    { "$/axes/c/homing/allow_single_axis" },
};

int  homed_axes = 0;
bool is_homed(int axis) {
    return homed_axes & (1 << axis);
}
void set_axis_homed(int axis) {
    homed_axes |= 1 << axis;
    request_redisplay();
}

void detect_homing_info() {
    clear_config_requests();
    for (int i = 0; i < homing_axis_count(); i++) {
        homing_cycles[i].init();
        homing_allows[i].init();
    }
    homed_axes = 0;
}
bool homes_in_all(int axis) {
    return homing_cycles[axis].known() && homing_cycles[axis].get() != 0;
}

bool can_home_individually(int axis) {
    return homing_allows[axis].known() && homing_allows[axis].get();
}

bool has_home_all_axes() {
    for (int axis = 0; axis < homing_axis_count(); ++axis) {
        if (homes_in_all(axis)) {
            return true;
        }
    }
    return false;
}

bool have_homing_info() {
    for (int i = 0; i < homing_axis_count(); ++i) {
        if (!homing_cycles[i].known() || !homing_allows[i].known()) {
            return false;
        }
    }
    return true;
}

class HomingScene : public Scene {
private:
    int _axis_to_home = -1;
    int _auto         = false;

    bool _allows[HOMING_N_AXIS];

public:
    HomingScene() : Scene("Home", 4) {}

    bool is_homing(int axis) {
        return _axis_to_home == -1 ? homes_in_all(axis) : (_axis_to_home == axis && can_home_individually(axis));
    }
    void onEntry(void* arg) override {
        if (state == Idle && _auto) {
            pop_scene();
        }
        const char* s = static_cast<const char*>(arg);
        _auto         = s && strcmp(s, "auto") == 0;
        if (!have_homing_info()) {
            schedule_action(detect_homing_info);
        }
    }

    void onStateChange(state_t old_state) override {
#ifdef AUTO_HOMING_RETURN
        if (old_state == Homing && state == Idle && _auto) {
            pop_scene();
        }
#endif
    }
    void onDialButtonPress() override { pop_scene(); }
    void onGreenButtonPress() override {
        if (state == Idle || state == Alarm) {
            if (_axis_to_home != -1) {
                if (can_home_individually(_axis_to_home)) {
                    send_linef("$H%c", axisNumToChar(_axis_to_home));
                }
            } else if (has_home_all_axes()) {
                send_line("$H");
            }
        } else if (state == Cycle) {
            fnc_realtime(FeedHold);
        } else if (state == Hold || state == DoorClosed) {
            fnc_realtime(CycleStart);
        }
    }
    void onRedButtonPress() override {
        if (state == Homing || state == Alarm) {
            fnc_realtime(Reset);
        }
    }

    void increment_axis_to_home() {
        do {
            ++_axis_to_home;
            if (_axis_to_home >= homing_axis_count()) {
                _axis_to_home = -1;
                return;
            }
        } while (!can_home_individually(_axis_to_home));
    }
    void decrement_axis_to_home() {
        do {
            if (_axis_to_home == -1) {
                _axis_to_home = homing_axis_count() - 1;
            } else if (--_axis_to_home < 0) {
                _axis_to_home = -1;
                return;
            }
        } while (!can_home_individually(_axis_to_home));
    }
    void onTouchClick() {
        if (state == Idle || state == Homing || state == Alarm) {
            increment_axis_to_home();
            reDisplay();
            ackBeep();
        }
    }

    void onEncoder(int delta) override {
        if (delta < 0) {
            decrement_axis_to_home();
        } else {
            increment_axis_to_home();
        }
        reDisplay();
    }
    void onDROChange() { reDisplay(); }  // also covers any status change

    void reDisplay() {
        background();
        drawMenuTitle(current_scene->name());
        drawStatus();

        const char* redLabel    = "";
        std::string grnLabel    = "";
        const char* orangeLabel = "";
        std::string green       = "Home ";

        int active_axes = homing_axis_count();
        int dro_height  = (active_axes <= 3) ? 32 : (active_axes == 4 ? 25 : 18);
        int dro_gap     = (active_axes <= 3) ? 33 : (active_axes == 6 ? 20 : dro_height + 5);
        int start_y     = 68;
        if (active_axes > 3) {
            static constexpr int list_top    = 64;
            static constexpr int list_bottom = 200;
            int list_height = dro_height + (active_axes - 1) * dro_gap;
            start_y         = list_top + (list_bottom - list_top - list_height) / 2;
        }
        fontnum_t font  = (active_axes <= 4) ? MEDIUM : SMALL;
        DRO dro(16, start_y, 210, dro_height, font, dro_gap);

        if (false && state == Homing) {
            for (int axis = 0; axis < active_axes; axis++) {
                dro.draw(axis, -1, true);
            }

        } else if (state == Idle || state == Homing || state == Alarm) {
            for (int axis = 0; axis < active_axes; ++axis) {
                dro.drawHoming(axis, is_homing(axis), is_homed(axis));
            }

#if 0
            int x      = 50;
            int y      = 65;
            int width  = display.width() - (x * 2);
            int height = 32;

            Stripe button(x, y, width, height, SMALL);
            button.draw("Home All", _axis_to_home == -1);
            y = button.y();  // LEDs start with the Home X button
            button.draw("Home X", _axis_to_home == 0);
            button.draw("Home Y", _axis_to_home == 1);
            button.draw("Home Z", _axis_to_home == 2);
            LED led(x - 16, y + height / 2, 10, button.gap());
            led.draw(myLimitSwitches[0]);
            led.draw(myLimitSwitches[1]);
            led.draw(myLimitSwitches[2]);
#endif

            if (state == Homing) {
                redLabel = "E-Stop";
            } else {
                if (state == Alarm && (strchr(myCtrlPins, 'D') == NULL)) {  // You can reset alarms if door is not active
                    redLabel = "Reset";
                }
                if (!have_homing_info()) {
                    orangeLabel = "Loading";
                } else if (_axis_to_home == -1) {
                    if (has_home_all_axes()) {
                        grnLabel = "Home All";
                    }
                } else if (can_home_individually(_axis_to_home)) {
                    grnLabel = "Home";
                    grnLabel += axisNumToChar(_axis_to_home);
                }
            }
        } else {
            centered_text("Invalid State", 105, WHITE, MEDIUM);
            centered_text("For Homing", 145, WHITE, MEDIUM);
            redLabel = "E-Stop";
            if (state == Cycle) {
                grnLabel = "Hold";
            } else if (state == Hold || state == DoorClosed) {
                grnLabel = "Resume";
            }
        }
        drawButtonLegends(redLabel, grnLabel.c_str(), orangeLabel[0] ? orangeLabel : "Back");

        refreshDisplay();
    }
};
HomingScene homingScene;
