#pragma once

#include "ofMain.h"
#include "sl_lidar.h"
#include <mutex>
#include <thread>
#include <atomic>

#ifndef NUM_POINTS
#define NUM_POINTS 8192
#endif

struct lidarPoint {
    float angle;
    float distance;
    float quality;
};

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
        void exit();
		
    private:
        lidarPoint lidarPoints[NUM_POINTS];

        void lidarLoop();
        std::mutex lidarMutex;
        std::thread lidarThread;
        std::atomic<bool> running;
};
