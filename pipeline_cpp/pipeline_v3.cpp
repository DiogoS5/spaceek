#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <opencv2/opencv.hpp>

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
static const int MAX_BLOB_PIXELS = (6*INTERP_FACTOR)*(6*INTERP_FACTOR);
static const float MIN_PEAK_TEMP  = 0.0f;     // ignore tiny warm specks

struct Blob {
    int id;
    float x,y;       // centroid
    bool countedEntry, countedExit;
    int confidence;
    int minRow,minCol,maxRow,maxCol;
};

// global buffers & state
float    origFrame[ORIG_NPIX];
float    frame[NPIX];
float    dynamicThreshold = 30.0f;
vector<Blob> tracks;
int      nextId = 1;

// estimate background by histogram‐mode
float estimateBackgroundTemp(const float* arr,int n) {
    const float minT=10.0f, maxT=50.0f;
    const int nbins=200;
    int hist[nbins]={0};
    float binw = (maxT-minT)/nbins;
    for(int i=0;i<n;i++){
        float v=arr[i];
        if(v<minT||v>maxT) continue;
        int b=int((v-minT)/binw);
        b = std::min(std::max(b, 0), nbins-1);  // replaced std::clamp
        hist[b]++;
    }
    int maxb=0;
    for(int i=1;i<nbins;i++) if(hist[i]>hist[maxb]) maxb=i;
    return minT + (maxb+0.5f)*binw;
}

// draw your thermal + tracks (unchanged)
void showFrame(const vector<Blob>& blobs) {
    cv::Mat raw(ROWS, COLS, CV_32F, frame);
    raw -= dynamicThreshold;
    double minVal, maxVal;
    cv::minMaxLoc(raw, &minVal, &maxVal);
    cv::Mat norm8;
    raw.convertTo(norm8, CV_8U,
                  255.0/(maxVal-minVal), 
                  -minVal*255.0/(maxVal-minVal));
    cv::Mat color;
    cv::applyColorMap(norm8, color, cv::COLORMAP_MAGMA);
    cv::Mat vis;
    cv::resize(color, vis, cv::Size(), 20,20, cv::INTER_NEAREST);

    // overlay numbers
    for(int r=0;r<ROWS;r++){
      for(int c=0;c<COLS;c++){
        float t = frame[r*COLS+c] + dynamicThreshold;
        char buf[16];
        snprintf(buf,sizeof(buf),"%.1f",round(t*10)/10.0f);
        cv::putText(vis, buf, {c*20+2, r*20+16},
                    cv::FONT_HERSHEY_PLAIN, 0.5,
                    cv::Scalar(0,0,0),1,cv::LINE_AA);
      }
    }
    // center line
    int cx = CROSS_COL*20;
    cv::line(vis, {cx,0},{cx,vis.rows},{255,255,255},2);

    // draw newly‐detected blobs
    for(auto &b:blobs){
      cv::Point c(int(b.x*20),int(b.y*20));
      cv::circle(vis,c,7,{0,255,255},-1);
      cv::putText(vis, to_string(b.id), c+cv::Point(5,10),
                  cv::FONT_HERSHEY_SIMPLEX,0.6,{0,255,255},1);
      cv::rectangle(vis,
        {b.minCol*20, b.minRow*20},
        {(b.maxCol+1)*20,(b.maxRow+1)*20},
        {0,255,0},1);
    }
    // draw ongoing tracks
    for(auto &t:tracks){
      cv::Point c(int(t.x*20),int(t.y*20));
      cv::circle(vis,c,5,{0,255,0},-1);
      cv::putText(vis, to_string(t.confidence),
                  c+cv::Point(5,-5),
                  cv::FONT_HERSHEY_SIMPLEX,0.6,
                  {0,255,0},1);
    }

    cv::imshow("Thermal Tracking", vis);
    if(cv::waitKey(0)==27) exit(0);
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
    cv::threshold(dist, sureFg, 0.5*maxD, 255, cv::THRESH_BINARY);
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
float dist2(float x1,float y1,float x2,float y2){
    float dx=x1-x2, dy=y1-y2; return dx*dx+dy*dy;
}

void trackAndReport(vector<Blob>& blobs) {
    vector<bool> used(blobs.size(),false), updated(tracks.size(),false);

    // match existing tracks → blobs
    for(int j=0;j<tracks.size();j++){
      float bestd=1e9; int bi=-1;
      for(int i=0;i<blobs.size();i++){
        if(used[i]) continue;
        float d = dist2(tracks[j].x,tracks[j].y,blobs[i].x,blobs[i].y);
        if(d<bestd){ bestd=d; bi=i; }
      }
      if(bi>=0 && bestd < (8.0*INTERP_FACTOR)*(8.0*INTERP_FACTOR)) {
        float prevX = tracks[j].x;
        tracks[j].x = blobs[bi].x;
        tracks[j].y = blobs[bi].y;
        used[bi]=true;
        updated[j]=true;
        tracks[j].confidence = min(tracks[j].confidence+1, 3);

        if(!tracks[j].countedEntry && prevX > CROSS_COL && tracks[j].x <= CROSS_COL) {
          cout<<"ENTRY "<<tracks[j].id<<endl;
          tracks[j].countedEntry=true;
          tracks[j].countedExit=false;
        }
        if(!tracks[j].countedExit && prevX <= CROSS_COL && tracks[j].x > CROSS_COL) {
          cout<<"EXIT "<<tracks[j].id<<endl;
          tracks[j].countedExit=true;
          tracks[j].countedEntry=false;
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
    // new blobs → fresh tracks
    for(int i=0;i<blobs.size();i++){
      if(!used[i]){
        blobs[i].id = nextId++;
        blobs[i].confidence = 0;
        tracks.push_back(blobs[i]);
      }
    }
}

void parseFrame(const string&s){
    int idx=0;
    string tok;
    stringstream ss(s);
    while(getline(ss,tok,',') && idx<ORIG_NPIX){
      origFrame[idx++] = atof(tok.c_str());
    }
}

int main(){
    ifstream fin("test3.csv");
    if(!fin){ cerr<<"Cannot open test3.csv\n"; return 1; }
    string line;
    while(getline(fin,line)){
      if(line.empty()) continue;
      parseFrame(line);

      // interpolate
      cv::Mat orig(ORIG_ROWS, ORIG_COLS, CV_32F, origFrame),
              interp;
      cv::resize(orig, interp,
                 {COLS,ROWS}, 0,0, cv::INTER_CUBIC);
      memcpy(frame, interp.data, sizeof(frame));

      // dynamic threshold
      float bg = estimateBackgroundTemp(frame, NPIX);
      //cout<<"Estimated BG: "<<bg<<endl;
      dynamicThreshold = bg + 4.0f;

      // find & track
      vector<Blob> blobs;
      findBlobsWatershed(blobs);
      trackAndReport(blobs);
      showFrame(blobs);
    }
    return 0;
}
