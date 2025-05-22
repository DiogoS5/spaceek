#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <opencv2/opencv.hpp>

using namespace std;

static const int ROWS = 12, COLS = 16;
static const int NPIX = ROWS * COLS;
static const float THRESHOLD = 30.0;        // adjust for your door temperature
static const int CROSS_COL = COLS/2;        // crossing line at halfway (vertical)

struct Blob {
  int id;
  float x, y;        // centroid in grid coordinates
  bool countedEntry, countedExit;
  int confidence = 0; // confidence level, start at 0, max 3. Blobs are deleted if confidence < 0
};

float frame[NPIX];
bool binmask[ROWS][COLS], visited[ROWS][COLS];
vector<Blob> tracks;
int nextId = 1;


void showFrame(const vector<Blob>& blobs) {
    // 1) Wrap your float array in a Mat
    cv::Mat raw(ROWS, COLS, CV_32F, frame);

    // 2) Find min/max
    raw -= THRESHOLD;  // subtract threshold if you want “zero” baseline
    double minVal, maxVal;
    cv::minMaxLoc(raw, &minVal, &maxVal);
    cv::Mat norm;
    raw.convertTo(norm, CV_8U,
                  255.0 / (maxVal - minVal),
                  -minVal * 255.0 / (maxVal - minVal));

    // 3) Apply a colormap
    cv::Mat color;
    cv::applyColorMap(norm, color, cv::COLORMAP_MAGMA);

    // 4) Upscale for visibility
    cv::Mat vis;
    cv::resize(color, vis, cv::Size(), 20, 20, cv::INTER_NEAREST);

    // Write temperature in black over each pixel, rounding to one decimal
    for (int r = 0; r < ROWS; ++r) {
      for (int c = 0; c < COLS; ++c) {
        float temp = frame[r * COLS + c] + THRESHOLD; // add back threshold offset
        float roundedTemp = round(temp * 10.0f) / 10.0f;
        char buf[8];
        snprintf(buf, sizeof(buf), "%.1f", roundedTemp);
        int x = c * 20 + 2;
        int y = r * 20 + 16;
        cv::putText(vis, buf, cv::Point(x, y), cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(0,0,0), 1, cv::LINE_AA);
      }
    }

    // 5) Draw crossing line
    int x = CROSS_COL * 20;
    cv::line(vis, {x,0}, {x,vis.rows}, {255,255,255}, 2);

    // 6) Draw blobs & track points
    for (auto &b : blobs) {
      cv::Point c(int(b.x*20), int(b.y*20));
      cv::circle(vis, c, 7, {0,255,255}, -1);
      cv::putText(vis, to_string(b.id), c + cv::Point(5,10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,255}, 1);
    }
    for (auto &t : tracks) {
      cv::Point c(int(t.x*20), int(t.y*20));
      cv::circle(vis, c, 5, {0,255,0}, -1);
      cv::putText(vis, to_string(t.confidence), c + cv::Point(5,-5),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, {0,255,0}, 1);
    }

    // 7) Show & wait
    cv::imshow("Thermal Tracking", vis);
    int key = cv::waitKey(0); // wait indefinitely for a key press
    if (key == 27) exit(0);
}

void parseFrame(const string &s) {
  int idx = 0;
  stringstream ss(s);
  string tok;
  while (getline(ss, tok, ',') && idx < NPIX) {
    frame[idx++] = atof(tok.c_str());
  }
}
// Function to find blobs in the thermal frame.
// The results are stored in a vector of Blob objects with centroids computed.
void findBlobs(vector<Blob>& blobs) {
  // Reset the visited flag for every cell in the grid.
  memset(visited, 0, sizeof(visited));
  
  // Create the binary mask for the current frame.
  // Each cell is set to true if the corresponding frame value is above the THRESHOLD.
  for (int r = 0; r < ROWS; ++r) {
    for (int c = 0; c < COLS; ++c) {
      binmask[r][c] = (frame[r * COLS + c] > THRESHOLD);
    }
  }

  // Define the relative row and column offsets for 4-connected neighborhood.
  int dr[4] = { -1, 1, 0, 0 };
  int dc[4] = { 0, 0, -1, 1 };

  // Loop through every cell in the grid.
  for (int r = 0; r < ROWS; ++r) {
    for (int c = 0; c < COLS; ++c) {
      // If the cell has not been visited and the value is above threshold,
      // start a flood-fill from this cell.
      if (!visited[r][c] && binmask[r][c]) {
        // Use a vector to implement the queue for flood-fill.
        vector<uint16_t> q;
        q.reserve(64); // reserve capacity for efficiency

        // Pack the initial cell position (row and column) into a 16-bit integer.
        // The higher 8 bits store the row value and the lower 8 bits store the column.
        q.push_back(r << 8 | c);

        // Mark the starting cell as visited.
        visited[r][c] = true;

        // qi: index for the current element in the flood-fill queue.
        // count: total number of connected cells found in this blob.
        int qi = 0, count = 0;

        // Variables to accumulate the sum of column and row indices,
        // which will later be used to compute the centroid.
        float sx = 0, sy = 0;

        // Process the queue until no more connected cells are found.
        while (qi < q.size()) {
          // Decode the packed cell position.
          int rr = q[qi] >> 8;       // extract row value (higher 8 bits)
          int cc = q[qi] & 0xFF;     // extract column value (lower 8 bits)
          ++qi;

          // Accumulate column and row positions.
          sx += cc;
          sy += rr;
          ++count;

          // Check all 4 neighboring cells.
          for (int d = 0; d < 4; ++d) {
            int nr = rr + dr[d];
            int nc = cc + dc[d];

            // If neighbor is within bounds, hasn't been visited,
            // and its value is above threshold (part of the blob), process it.
            if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS
                && !visited[nr][nc] && binmask[nr][nc]) {
              // Mark neighbor as visited.
              visited[nr][nc] = true;
              // Pack the neighbor position and add to the flood-fill queue.
              q.push_back(nr << 8 | nc);
            }
          }
        }

        // After finishing the flood-fill for the blob, compute its centroid.
        //if (count >= 9) { // Only keep blobs with at least 9 pixels
          Blob b{};
          b.id = 0; // the blob's id will be assigned later when tracking.
          // Centroid is computed as the average column (x) and row (y) positions.
          b.x = sx / count;
          b.y = sy / count;
          b.countedEntry = b.countedExit = false;

          // Add the blob object to the blob vector.
          blobs.push_back(b);
        //}
      }
    }
  }
}

float dist2(float x1,float y1,float x2,float y2){
  float dx=x1-x2, dy=y1-y2; return dx*dx+dy*dy;
}

void trackAndReport(vector<Blob>& blobs) {
  vector<bool> used(blobs.size(), false);
  vector<bool> trackUpdated(tracks.size(), false);
  
  // For each existing track in our global 'tracks' container...
  for (int j = 0; j < tracks.size(); j++) {
    float bestd = 1e9;
    int bi = -1;
    for (int i = 0; i < blobs.size(); ++i) {
      if (used[i])
        continue;
      float d = dist2(tracks[j].x, tracks[j].y, blobs[i].x, blobs[i].y);
      if (d < bestd) {
        bestd = d;
        bi = i;
      }
    }
    if (bi >= 0 && bestd < 25) { // within 3-pixel radius
      float prevX = tracks[j].x;
      tracks[j].x = blobs[bi].x;
      tracks[j].y = blobs[bi].y;
      used[bi] = true;
      trackUpdated[j] = true;
      tracks[j].confidence = min(tracks[j].confidence + 1, 3); // increase confidence, without exceeding 3
      std::cout << "PREV POS" << prevX << " CURR POS" << tracks[j].x << std::endl;
      // Check crossing for ENTRY: right-to-left move.
      if (!tracks[j].countedEntry && prevX > CROSS_COL && tracks[j].x <= CROSS_COL) {
        cout << "ENTRY " << tracks[j].id << endl;
        tracks[j].countedEntry = true;
        tracks[j].countedExit = false; // reset exit count
      }
      // Check crossing for EXIT: left-to-right move.
      if (!tracks[j].countedExit && prevX <= CROSS_COL && tracks[j].x > CROSS_COL) {
        cout << "EXIT " << tracks[j].id << endl;
        tracks[j].countedExit = true;
        tracks[j].countedEntry = false; // reset entry count
      }
    }
  }
  
  // Lower confidence for tracks not updated
  for (int k = 0; k < trackUpdated.size(); ++k) {
    if (!trackUpdated[k]) {
      tracks[k].confidence--;
    }
  }
  // Remove tracks with confidence 0 or on border
  for (int k = trackUpdated.size() - 1; k >= 0; k--) {
    if (tracks[k].confidence <= 0 || (tracks[k].x < 1 || tracks[k].x > COLS - 2)) {
      tracks.erase(tracks.begin() + k);
    }
  }
  
  // For every blob that was not used to update an existing track, create a new track.
  for (int i = 0; i < blobs.size(); ++i) {
    if (!used[i]) {
      blobs[i].id = nextId++;
      blobs[i].confidence = 0; // start new track with confidence 0
      tracks.push_back(blobs[i]);
    }
  }
}

int main() {
  ifstream fin("test1.csv");
  if (!fin) {
    cerr << "Failed to open test1.csv" << endl;
    return 1;
  }
  string line;
  while (getline(fin, line)) {
    if (line.empty()) continue;
    parseFrame(line);
    vector<Blob> blobs;
    findBlobs(blobs);
    trackAndReport(blobs);
    showFrame(blobs);
  }
  return 0;
}