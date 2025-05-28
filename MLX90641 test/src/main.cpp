/*
    Output the temperature readings to all pixels to be read by a Processing visualizer
*/

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

#define USE_MLX90641

#ifndef USE_MLX90641
    #include "MLX90640_API.h"
#else
    #include "MLX90641_API.h"
#endif

#include "MLX9064X_I2C_Driver.h"

const byte MLX90641_address = 0x33; //Default 7-bit unshifted address of the MLX90641
#define TA_SHIFT 8 //Default shift for MLX90641 in open air

uint16_t eeMLX90641[832];
float MLX90641To[192];
uint16_t MLX90641Frame[242];
paramsMLX90641 MLX90641;
int errorno = 0;

#if defined(ARDUINO_ARCH_AVR)
    #define debug  Serial

#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
    #define debug  Serial
#else
    #define debug  Serial
#endif

boolean isConnected();

#define ORIG_ROWS 12
#define ORIG_COLS 16
#define INTERP_FACTOR 2
#define ROWS (ORIG_ROWS * INTERP_FACTOR)
#define COLS (ORIG_COLS * INTERP_FACTOR)
#define CROSS_COL (COLS / 2)
#define MIN_BLOB_PIXELS (2 * INTERP_FACTOR * 2 * INTERP_FACTOR)
#define MAX_BLOB_PIXELS (6 * INTERP_FACTOR * 6 * INTERP_FACTOR)
#define MIN_PEAK_TEMP 0.0f

struct Blob {
    int id;
    float x, y;
    bool countedEntry, countedExit;
    int confidence;
};

float origFrame[ORIG_ROWS * ORIG_COLS];
float frame[ROWS * COLS];
float dynamicThreshold = 30.0f;
vector<Blob> tracks;
int nextId = 1;

float estimateBackgroundTemp(const float* arr, int n) {
    const float minT = 10.0f, maxT = 50.0f;
    const int nbins = 200;
    int hist[nbins] = {0};
    float binw = (maxT - minT) / nbins;
    for (int i = 0; i < n; i++) {
        float v = arr[i];
        if (v < minT || v > maxT) continue;
        int b = int((v - minT) / binw);
        b = min(max(b, 0), nbins - 1);
        hist[b]++;
    }
    int maxb = 0;
    for (int i = 1; i < nbins; i++) if (hist[i] > hist[maxb]) maxb = i;
    return minT + (maxb + 0.5f) * binw;
}

void findBlobs(vector<Blob>& blobs) {
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (frame[r * COLS + c] > dynamicThreshold) {
                Blob b;
                b.id = nextId++;
                b.x = c;
                b.y = r;
                b.countedEntry = b.countedExit = false;
                b.confidence = 0;
                blobs.push_back(b);
            }
        }
    }
}

float dist2(float x1, float y1, float x2, float y2) {
    float dx = x1 - x2, dy = y1 - y2;
    return dx * dx + dy * dy;
}

void trackAndReport(vector<Blob>& blobs) {
    vector<bool> used(blobs.size(), false), updated(tracks.size(), false);

    for (int j = 0; j < tracks.size(); j++) {
        float bestd = 1e9;
        int bi = -1;
        for (int i = 0; i < blobs.size(); i++) {
            if (used[i]) continue;
            float d = dist2(tracks[j].x, tracks[j].y, blobs[i].x, blobs[i].y);
            if (d < bestd) {
                bestd = d;
                bi = i;
            }
        }
        if (bi >= 0 && bestd < (8.0 * INTERP_FACTOR) * (8.0 * INTERP_FACTOR)) {
            float prevX = tracks[j].x;
            tracks[j].x = blobs[bi].x;
            tracks[j].y = blobs[bi].y;
            used[bi] = true;
            updated[j] = true;
            tracks[j].confidence = min(tracks[j].confidence + 1, 3);

            if (!tracks[j].countedEntry && prevX > CROSS_COL && tracks[j].x <= CROSS_COL) {
                Serial.println("ENTRY " + String(tracks[j].id));
                tracks[j].countedEntry = true;
                tracks[j].countedExit = false;
            }
            if (!tracks[j].countedExit && prevX <= CROSS_COL && tracks[j].x > CROSS_COL) {
                Serial.println("EXIT " + String(tracks[j].id));
                tracks[j].countedExit = true;
                tracks[j].countedEntry = false;
            }
        }
    }

    for (int k = int(tracks.size()) - 1; k >= 0; k--) {
        if (tracks[k].confidence <= 0 || tracks[k].x < 1 || tracks[k].x > COLS - 2)
            tracks.erase(tracks.begin() + k);
    }

    for (int i = 0; i < blobs.size(); i++) {
        if (!used[i]) {
            blobs[i].id = nextId++;
            blobs[i].confidence = 0;
            tracks.push_back(blobs[i]);
        }
    }
}

void parseFrame(const String& s) {
    int idx = 0;
    char* tok = strtok((char*)s.c_str(), ",");
    while (tok != NULL && idx < ORIG_ROWS * ORIG_COLS) {
        origFrame[idx++] = atof(tok);
        tok = strtok(NULL, ",");
    }
}

void setup() {
    Wire.begin();
    Wire.setClock(400000); //Increase I2C clock speed to 400kHz

    debug.begin(115200); //Fast debug as possible

    while (!debug); //Wait for user to open terminal
    //debug.println("MLX90640 IR Array Example");

    if (isConnected() == false) {
        debug.println("MLX90641 not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
    }
    //Get device parameters - We only have to do this once
    int status;
    status = MLX90641_DumpEE(MLX90641_address, eeMLX90641);
    errorno = status;//MLX90641_CheckEEPROMValid(eeMLX90641);//eeMLX90641[10] & 0x0040;//
    
    if (status != 0) {
        debug.println("Failed to load system parameters");
       while(1);
    }

    status = MLX90641_ExtractParameters(eeMLX90641, &MLX90641);
    //errorno = status;
    if (status != 0) {
        debug.println("Parameter extraction failed");
        while(1);
    }

    //Once params are extracted, we can release eeMLX90641 array

    //MLX90641_SetRefreshRate(MLX90641_address, 0x02); //Set rate to 2Hz
    MLX90641_SetRefreshRate(MLX90641_address, 0x04); //Set rate to 4Hz
    //MLX90641_SetRefreshRate(MLX90641_address, 0x07); //Set rate to 64Hz    
}

void bilinearInterpolate(const float* src, int srcRows, int srcCols, float* dst, int dstRows, int dstCols) {
    float rowScale = float(srcRows - 1) / float(dstRows - 1);
    float colScale = float(srcCols - 1) / float(dstCols - 1);
    for (int r = 0; r < dstRows; r++) {
        float srcR = r * rowScale;
        int r0 = int(srcR);
        int r1 = min(r0 + 1, srcRows - 1);
        float fr = srcR - r0;
        for (int c = 0; c < dstCols; c++) {
            float srcC = c * colScale;
            int c0 = int(srcC);
            int c1 = min(c0 + 1, srcCols - 1);
            float fc = srcC - c0;
            float v00 = src[r0 * srcCols + c0];
            float v01 = src[r0 * srcCols + c1];
            float v10 = src[r1 * srcCols + c0];
            float v11 = src[r1 * srcCols + c1];
            float v0 = v00 + (v01 - v00) * fc;
            float v1 = v10 + (v11 - v10) * fc;
            dst[r * dstCols + c] = v0 + (v1 - v0) * fr;
        }
    }
}

void loop() {
    long startTime = millis();
    for (byte x = 0 ; x < 2 ; x++) {
        int status = MLX90641_GetFrameData(MLX90641_address, MLX90641Frame);
        float vdd = MLX90641_GetVdd(MLX90641Frame, &MLX90641);
        float Ta = MLX90641_GetTa(MLX90641Frame, &MLX90641);
        float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
        float emissivity = 0.95;
        MLX90641_CalculateTo(MLX90641Frame, &MLX90641, emissivity, tr, MLX90641To);
    }
    long stopTime = millis();

    // 1. Copy MLX90641To to origFrame
    for (int i = 0; i < ORIG_ROWS * ORIG_COLS; i++) {
        origFrame[i] = MLX90641To[i];
    }
    // 2. Interpolate to higher resolution
    bilinearInterpolate(origFrame, ORIG_ROWS, ORIG_COLS, frame, ROWS, COLS);
    // 3. Estimate background and set dynamic threshold
    float bg = estimateBackgroundTemp(frame, ROWS * COLS);
    dynamicThreshold = bg + 4.0f;
    // 4. Find blobs
    vector<Blob> blobs;
    findBlobs(blobs);
    // 5. Track and report entries/exits
    trackAndReport(blobs);
    // Optional: print debug info
    // debug.print("BG: "); debug.println(bg, 2);
    // debug.print("Blobs: "); debug.println(blobs.size());
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected() {
    Wire.beginTransmission((uint8_t)MLX90641_address);
    if (Wire.endTransmission() != 0) {
        return (false);    //Sensor did not ACK
    }
    return (true);
}