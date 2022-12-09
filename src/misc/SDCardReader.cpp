#include "misc/SDCardReader.h"

using namespace pros;

std::vector<std::vector<Position*>> SDCardReader::readSDCardData(std::string filename) {
    std::string path = "/usd/" + filename;
    char char_array[path.length() + 1];
    strcpy(char_array, path.c_str());
    FILE* fp = fopen(char_array, "r");

    if(!fp) {
        perror(char_array);
        exit(1);
    }

    fseek(fp, 0L, SEEK_END);
    long lSize = ftell(fp);
    rewind(fp);

    pros::lcd::initialize();
    char *buffer;
    /* allocate memory for entire content */
    buffer = (char*) calloc(1, lSize + 1);
    if(!buffer) {
        fclose(fp);
        pros::lcd::print(0, "Memory allocation failed");
        // exit(1);
    }

    /* copy the file into the buffer */
    if(fread(buffer, lSize, 1, fp) != 1) {
        fclose(fp);
        free(buffer);
        pros::lcd::print(0, "File read failed");
        // exit(1);
    }

    std::string s = "";
    std::vector<std::vector<Position*>> segments;
    std::vector<Position*> segmentVector;
    int currInd = 0;
    for (int i = 0; i < lSize; i++) {
        s += buffer[i];
        if (i != lSize - 1 && buffer[i] == 13 && buffer[i + 1] == 10) {
            if (s == "") { // empty line indicating segment break
                segments.push_back(segmentVector);
                segmentVector.clear();
            } else { // add Position to current line segment
                int commaIndex = s.find(",");
                float x = atof(s.substr(0, commaIndex).c_str());
                float y = atof(s.substr(commaIndex + 1, s.length() - commaIndex + 1).c_str());
                Position position = { x = x, y = y };
                segmentVector.push_back(&position);
                currInd++;
                s = "";
            }
        }
    }

    free(buffer);

    return segments;
}