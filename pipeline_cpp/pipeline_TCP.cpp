#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>

using namespace std;

// New constants for interpolation
static const int ORIG_ROWS    = 12;
static const int ORIG_COLS    = 16;
static const int INTERP_FACTOR= 2;
static const int ROWS         = ORIG_ROWS * INTERP_FACTOR;
static const int COLS         = ORIG_COLS * INTERP_FACTOR;
static const int NPIX         = ROWS * COLS;
static const int ORIG_NPIX    = ORIG_ROWS * ORIG_COLS;
static const int CROSS_COL    = COLS/2;        // crossing line at halfway (vertical)
static const int MIN_BLOB_PIXELS = (2*INTERP_FACTOR)*(2*INTERP_FACTOR);
static const int MAX_BLOB_PIXELS = (5*INTERP_FACTOR)*(5*INTERP_FACTOR);
static const float MIN_PEAK_TEMP  = 0.0f;     // ignore tiny warm specks

int occupancy = 2;
int n_mov = 0;

struct Blob {
    int id;
    float x,y;            // centroid
    float vx,vy;          // velocity components
    bool countedEntry, countedExit;
    int confidence;
    int minRow,minCol,maxRow,maxCol;
};

// Global buffers & state
float    origFrame[ORIG_NPIX];
float    frame[NPIX];
float    dynamicThreshold = 30.0f;
vector<Blob> tracks;
int      nextId = 1;

// TCP Configuration
const std::string LOCAL_IP = "172.20.10.4"; // Server IP, check by running `ifconfig` on the terminal of the server
const int TCP_PORT = 5002;                  // Server Listening Port

int setupTCPSocket() {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        cerr << "Socket creation failed" << endl;
        exit(EXIT_FAILURE);
    }

    // Reuse address
    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        cerr << "Setsockopt failed" << endl;
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(TCP_PORT);
    
    if (inet_pton(AF_INET, LOCAL_IP.c_str(), &servaddr.sin_addr) <= 0) {
        cerr << "Invalid IP address: " << LOCAL_IP << endl;
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (bind(sockfd, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        cerr << "Bind failed on " << LOCAL_IP << ":" << TCP_PORT << endl;
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    if (listen(sockfd, 1) < 0) {
        cerr << "Listen failed" << endl;
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    cout << "TCP server listening on " << LOCAL_IP << ":" << TCP_PORT << endl;
    return sockfd;
}

int acceptClient(int server_fd) {
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    
    cout << "Waiting for client connection..." << endl;
    int client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
    if (client_fd < 0) {
        cerr << "Accept failed" << endl;
        return -1;
    }

    char client_ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
    cout << "Client connected from " << client_ip << endl;

    return client_fd;
}

float estimateBackgroundTemp(const float* arr, int n) {
    const float minT = 10.0f, maxT = 50.0f;
    const int nbins = 200;
    int hist[nbins] = {0};
    float binw = (maxT - minT) / nbins;
    for(int i = 0; i < n; i++){
        float v = arr[i];
        if(v < minT || v > maxT) continue;
        int b = int((v - minT) / binw);
        b = std::min(std::max(b, 0), nbins-1);
        hist[b]++;
    }

    //pick 25th percentile
    int total = 0;
    for(int i = 0; i < nbins; i++) total += hist[i];
    int target = total / 4;
    int cum = 0, qb = 0;
    for(int i = 0; i < nbins; i++){
        cum += hist[i];
        if(cum >= target){ qb = i; break; }
    }
    return minT + (qb + 0.5f) * binw;
}

// Draw your thermal + tracks
void showFrame(const vector<Blob>& blobs) {
    // 1. Prepare thermal image
    cv::Mat thermal(ROWS, COLS, CV_32F, frame);
    thermal -= dynamicThreshold; // Apply threshold
    
    // 2. Normalize to 0-255
    cv::Mat normalized;
    double minVal, maxVal;
    cv::minMaxLoc(thermal, &minVal, &maxVal);
    thermal.convertTo(normalized, CV_8U, 255.0/(maxVal-minVal), -minVal*255.0/(maxVal-minVal));
    
    // 3. Apply color map (MAGMA = black-yellow-red-white)
    cv::Mat colorMap;
    cv::applyColorMap(normalized, colorMap, cv::COLORMAP_MAGMA);
    
    // 4. Upscale for visibility (20x zoom)
    cv::Mat display;
    cv::resize(colorMap, display, cv::Size(), 20, 20, cv::INTER_NEAREST);
    
    // 5. Draw center line (white)
    int centerX = (COLS/2) * 20;
    cv::line(display, 
             cv::Point(centerX, 0), 
             cv::Point(centerX, display.rows), 
             cv::Scalar(255, 255, 255), 2);
    
    // 6. Draw temperature values (black text)
    for(int r = 0; r < ROWS; r++) {
        for(int c = 0; c < COLS; c++) {
            float temp = frame[r*COLS + c] + dynamicThreshold;
            std::string text = std::to_string(int(round(temp)));
            cv::putText(display, text, 
                        cv::Point(c*20 + 2, r*20 + 16), // Position
                        cv::FONT_HERSHEY_PLAIN, 0.5, 
                        cv::Scalar(0, 0, 0), 1); // Black text
        }
    }
    
    // 7. Draw detected blobs (yellow circles)
    for(const auto& blob : blobs) {
        cv::Point center(int(blob.x * 20), int(blob.y * 20));
        // Circle with ID
        cv::circle(display, center, 7, cv::Scalar(0, 255, 255), -1);
        cv::putText(display, std::to_string(blob.id), 
                    center + cv::Point(5, 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                    cv::Scalar(0, 255, 255), 1);
        // Bounding box (green)
        cv::rectangle(display,
                     cv::Point(blob.minCol * 20, blob.minRow * 20),
                     cv::Point((blob.maxCol + 1) * 20, (blob.maxRow + 1) * 20),
                     cv::Scalar(0, 255, 0), 1);
    }
    
    // 8. Draw tracked objects (green circles)
    for(const auto& track : tracks) {
        cv::Point center(int(track.x * 20), int(track.y * 20));
        cv::circle(display, center, 5, cv::Scalar(0, 255, 0), -1);
        cv::putText(display, std::to_string(track.confidence),
                    center + cv::Point(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 0), 1);
    }
    
    // 9. Display with auto-update
    cv::imshow("Thermal Tracking", display);
    cv::waitKey(1); // Non-blocking 1ms delay (crucial for real-time)
}

// --- New: marker‐based watershed blob finder ---
void findBlobsWatershed(vector<Blob>& blobs) {
    // 1) build a binary mask at dynamicThreshold
    cv::Mat bin(ROWS, COLS, CV_8U);
    for(int r=0;r<ROWS;r++){
      for(int c=0;c<COLS;c++){
        bin.at<uchar>(r,c) = (frame[r*COLS+c] > dynamicThreshold ? 255 : 0);
      }
    }

    // 2) sure background = dilate
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,{3,3});
    cv::Mat sureBg;
    cv::dilate(bin, sureBg, kernel, {}, 3);

    // 3) distance transform on the mask
    cv::Mat dist;
    cv::distanceTransform(bin, dist, cv::DIST_L2, 5);

    // 4) threshold dist to get sure foreground markers
    double maxD;
    cv::minMaxLoc(dist, nullptr, &maxD);
    cv::Mat sureFg;
    cv::threshold(dist, sureFg, 0.4*maxD, 255, cv::THRESH_BINARY);
    sureFg.convertTo(sureFg, CV_8U);

    // 5) unknown region = background minus foreground
    cv::Mat unknown = sureBg - sureFg;

    // 6) label the foreground markers
    cv::Mat markers;
    int nMarkers = cv::connectedComponents(sureFg, markers);
    // shift so background=1, objects = 2…nMarkers
    markers += 1;
    // mark unknown with zero
    markers.setTo(0, unknown);

    // 7) prepare a 3-channel image for watershed
    cv::Mat raw(ROWS, COLS, CV_32F, frame);
    raw -= dynamicThreshold;
    double mn, mx;
    cv::minMaxLoc(raw, &mn, &mx);
    cv::Mat norm8;
    raw.convertTo(norm8, CV_8U,
                  255.0/(mx-mn),
                  -mn*255.0/(mx-mn));
    cv::Mat color;
    cv::cvtColor(norm8, color, cv::COLOR_GRAY2BGR);

    // 8) run watershed
    cv::watershed(color, markers);

    // 9) extract each label as its own Blob
    for(int lbl = 2; lbl <= nMarkers; lbl++){
      float sx=0, sy=0;
      int count=0, minR=ROWS, maxR=0, minC=COLS, maxC=0;
      float peak=-1e9;
      for(int r=0;r<ROWS;r++){
        for(int c=0;c<COLS;c++){
          if(markers.at<int>(r,c) != lbl) continue;
          count++;
          sx += c; sy += r;
          float t = frame[r*COLS+c];
          peak = max(peak, t);
          minR = min(minR, r); maxR = max(maxR, r);
          minC = min(minC, c); maxC = max(maxC, c);
        }
      }
      if(count < MIN_BLOB_PIXELS || count > MAX_BLOB_PIXELS) continue;
      if(peak < MIN_PEAK_TEMP) continue;
      Blob b;
      b.id = 0;
      b.x  = sx/count;
      b.y  = sy/count;
      b.minRow = minR;
      b.maxRow = maxR;
      b.minCol = minC;
      b.maxCol = maxC;
      b.countedEntry = b.countedExit = false;
      b.confidence = 0;
      blobs.push_back(b);
    }
}

// rest of your tracker unchanged
float dist2(float x1,float y1,float x2,float y2) {
    float dx=x1-x2, dy=y1-y2; return dx*dx+dy*dy;
}

void trackAndReport(vector<Blob>& blobs) {
    vector<bool> used(blobs.size(),false), updated(tracks.size(),false);

    // 1) Match existing tracks → blobs using velocity‐based prediction
    for(int j=0;j<tracks.size();j++){
      float predX = tracks[j].x + tracks[j].vx;
      float predY = tracks[j].y + tracks[j].vy;
      float bestd=1e9; int bi=-1;
      for(int i=0;i<blobs.size();i++){
        if(used[i]) continue;
        float d = dist2(predX,predY, blobs[i].x, blobs[i].y);
        if(d<bestd){ bestd=d; bi=i; }
      }
      if(bi>=0 && bestd < (6.0*INTERP_FACTOR)*(6.0*INTERP_FACTOR)) {
        float prevX = tracks[j].x, prevY = tracks[j].y;
        tracks[j].x = blobs[bi].x;
        tracks[j].y = blobs[bi].y;
        // 2) Update velocity
        tracks[j].vx = tracks[j].x - prevX;
        tracks[j].vy = tracks[j].y - prevY;
        used[bi]=true; updated[j]=true;
        // 3) Increase confidence up to 5
        tracks[j].confidence = min(tracks[j].confidence+1, 5);
        // 4) Penalize if in edge quarter and moving away
        int ex1=COLS/5, ex2=4*COLS/5;
        if((tracks[j].x<ex1 && tracks[j].vx<2) || (tracks[j].x>ex2 && tracks[j].vx>2)) {
          tracks[j].confidence = max(tracks[j].confidence - 2, 0);
        }
        // 5) Entry/exit logic unchanged
        if(!tracks[j].countedEntry && prevX > CROSS_COL && tracks[j].x <= CROSS_COL) {
          cout<<"ENTRY "<<tracks[j].id<<endl;
          tracks[j].countedEntry=true; tracks[j].countedExit=false;

          occupancy++;
          cout << "Current count: " << occupancy << endl;
        }
        if(!tracks[j].countedExit && prevX <= CROSS_COL && tracks[j].x > CROSS_COL) {
          cout<<"EXIT "<<tracks[j].id<<endl;
          tracks[j].countedExit=true; tracks[j].countedEntry=false;

          occupancy--;
          cout << "Current count: " << occupancy << endl;
        }
      }
    }

    // degrade old tracks
    for(int k=0;k<updated.size();k++){
      if(!updated[k]) tracks[k].confidence--;
    }
    // remove dead tracks
    for(int k=int(tracks.size())-1;k>=0;k--){
      if(tracks[k].confidence<=0
       || tracks[k].x<1 || tracks[k].x>COLS-2)
        tracks.erase(tracks.begin()+k);
    }
    // 6) New blobs → fresh tracks (init velocity to zero)
    for(int i=0;i<blobs.size();i++){
      if(!used[i]){ // new blobs only allowed in the outer quarters
        blobs[i].id = nextId++;
        blobs[i].confidence = 0;
        blobs[i].vx = blobs[i].vy = 0;   // ← init velocity
        tracks.push_back(blobs[i]);
      }
    }
}

void parseFrame(const string&s) {
    int idx=0;
    string tok;
    stringstream ss(s);
    while(getline(ss,tok,',') && idx<ORIG_NPIX){
      origFrame[idx++] = atof(tok.c_str());
    }
}

int main() {
    int server_fd = setupTCPSocket();
    int client_fd = acceptClient(server_fd);
    if (client_fd < 0) return 1;
    float frameData[ORIG_NPIX];
    
    // For frame rate control
    using namespace std::chrono;
    auto last_frame = steady_clock::now();
    constexpr int TARGET_FPS = 16;  // Match your ESP32's frame rate
    
    while (true) {
        // 1. Receive frame
        int total_received = 0;
        while (total_received < sizeof(frameData))
        {
            int n = recv(client_fd, (char*)frameData + total_received, sizeof(frameData) - total_received, 0);
            
            if (n <= 0)
            {
                cerr << "Connection lost or error, waiting for new client..." << endl;
                close(client_fd);
                client_fd = acceptClient(server_fd);
                if (client_fd < 0) return 1;
                total_received = 0;
                continue;
            }
            total_received += n;
        }

        // 2. Process frame
        memcpy(origFrame, frameData, sizeof(frameData));
        cv::Mat orig(ORIG_ROWS, ORIG_COLS, CV_32F, origFrame);
        cv::Mat interp;
        cv::resize(orig, interp, {COLS,ROWS}, 0, 0, cv::INTER_CUBIC);

        /*
        for (int c = 0; c < interp.cols; ++c) {
            float colSum = 0;
            for (int r = 0; r < interp.rows; ++r)
            {
                colSum += interp.at<float>(r, c);
            }
            float colAvg = colSum / interp.rows;
            for (int r = 0; r < interp.rows; ++r)
            {
                interp.at<float>(r, c) -= colAvg;
            }
        }
        */

        /*
        for (int c = 0; c < interp.cols; ++c) {
            float colAvg = cv::mean(interp.col(c))[0];
            interp.col(c) -= colAvg;
        }
        */

        // Image Denoising
        //cv::medianBlur(interp, interp, 5);  // Removes salt-and-pepper noise
        //cv::blur(interp, interp, cv::Size(3, 1));  // Horizontal smoothing
        //cv::Mat corrected = interp.clone();  // Make a copy you can modify
        //cv::Mat kernel = (cv::Mat_<float>(1, 3) << -0.25, 0.5, -0.25);
        //cv::filter2D(corrected, corrected, -1, kernel);
        //cv::medianBlur(interp, interp, 3);  // 3x3 median filter

        memcpy(frame, interp.data, sizeof(frame));

        float bg = estimateBackgroundTemp(frame, NPIX);
        //printf("%f\n", bg);
        //float bg = 28.1f;
        dynamicThreshold = bg + 3.0f;
        
        vector<Blob> blobs;
        findBlobsWatershed(blobs);
        trackAndReport(blobs);
        
        // 3. Auto-updating display
        showFrame(blobs);
        
        // 4. Frame rate control
        auto now = steady_clock::now();
        auto elapsed = duration_cast<milliseconds>(now - last_frame).count();
        int delay = (1000/TARGET_FPS) - elapsed;
        if (delay > 0) std::this_thread::sleep_for(milliseconds(delay));
        last_frame = now;
    }
    
    close(client_fd);
    close(server_fd);

    return 0;
}