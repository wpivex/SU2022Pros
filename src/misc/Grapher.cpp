#include "misc/Grapher.h"
#include "main.h"
#include <string>

namespace graphy {

AsyncGrapher::AsyncGrapher(double minValue, double maxValue, const okapi::QTime &rate):
    min(minValue),
    max(maxValue)
 {
    this->refreshRate = rate;
    cnt = 0;
}

void AsyncGrapher::addDataType(const std::string &name, const uint32_t color) {
    if (cnt > MAX_DATA) {
        std::runtime_error("Error: max number of data is 14");
    } else {
        cnt++;
        container.insert({name, {0.0}});
        colors.insert({name, color});
    }
}

void AsyncGrapher::update(const std::string &name, double val) {
    val = (val - min) / (max - min); // bound between (0,1)
    container[name].push_back(val);
    if (container[name].size() > MAX_CACHE_SIZE) {
        container[name].erase(container[name].begin());
    }
}

void AsyncGrapher::setRefreshRate(const okapi::QTime &rate) {
    this->refreshRate = rate;
}

okapi::QTime AsyncGrapher::getRefreshRate() {
    return this->refreshRate;
}

void AsyncGrapher::loop() {
    while (true) {
        // Reset screen
        pros::screen::erase();

        // Draw graph
        pros::screen::set_pen(COLOR_WHITE);
        

        std::string mins = std::to_string(min);
        std::string maxs = std::to_string(max);
        pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, 5, GRAPH_BOTTOM, mins.c_str());
        pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, 5, GRAPH_TOP, maxs.c_str());


        pros::screen::draw_line(GRAPH_LEFT, GRAPH_TOP, GRAPH_LEFT, GRAPH_BOTTOM);
        pros::screen::draw_line(
          GRAPH_LEFT, GRAPH_BOTTOM, GRAPH_LEFT + MAX_CACHE_SIZE, GRAPH_BOTTOM);

        int indexLine = 0;
        for (const auto &item : container) {
            pros::screen::set_pen(colors[item.first]);
            pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL,
                                GRAPH_LEFT + MAX_CACHE_SIZE,
                                (++indexLine) * 14 + 30,
                                item.first.c_str());
            for (int i = 0; i < item.second.size()-1; i++) {
                double val1 = item.second[i] * (GRAPH_BOTTOM - GRAPH_TOP);
                double val2 = item.second[i + 1] * (GRAPH_BOTTOM - GRAPH_TOP);
                pros::screen::draw_line(
                  GRAPH_LEFT + i, GRAPH_BOTTOM - val1, GRAPH_LEFT + i + 1, GRAPH_BOTTOM - val2);
            }
        }

        pros::delay(refreshRate.convert(okapi::millisecond));
    }
}

}  // namespace graph