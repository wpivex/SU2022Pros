#pragma once

#include <vector>
#include "main.h"

class Logger {

private:
	std::vector<lv_obj_t*> text;
    int row = 0;

public:

    Logger() {
        clear();
    }

    template <class ... Args>
    void logAt(int x, int y, const char *f, Args ... args) {
        // Generate formatted text from arguemnts
        char formattedText[200];
        sprintf(formattedText, f, args...);

        printf(formattedText);
        printf("\n%d %d\n", x, y);

        pros::screen::set_pen(COLOR_WHITE);
        std::string a("Test");
        pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, x, y, a.c_str());
    }

    template <class ... Args>
    void log(const char *f, Args ... args) {

        logAt(5, 5 + row*20, f, args...);
        row++;
	}

    void clear() {
        pros::screen::erase();
    }
};