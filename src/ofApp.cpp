#include <signal.h>

#include "ofApp.h"
#include "sl_lidar.h" 
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


bool checkSLAMTECLIDARHealth(sl::ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

static inline void delay(sl_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

//--------------------------------------------------------------
void ofApp::setup(){
    running = true;
    lidarThread = std::thread(&ofApp::lidarLoop, this);
    ofSetFrameRate(60);
}

//--------------------------------------------------------------
void ofApp::update(){
}

void ofApp::exit() {
    running = false;
    if (lidarThread.joinable()) {
        lidarThread.join();
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofPushMatrix();
    ofTranslate(ofVec2f(ofGetWidth(), ofGetHeight())/2.f);

    for (int pos = 0; pos < NUM_POINTS ; ++pos) {
        std::lock_guard<std::mutex> lock(lidarMutex);
        auto lidarPoint = lidarPoints[pos];
        if(lidarPoint.quality > 0) {
            ofVec2f pos = ofVec2f(lidarPoint.distance, 0).getRotated(lidarPoint.angle);
            ofDrawCircle(pos, 5);
        }
    }

    ofPopMatrix();
}

void ofApp::lidarLoop(){
    sl::ILidarDriver * drv;
    sl_result op_result;
    sl_lidar_response_measurement_node_hq_t nodes[8192];

    // create the driver instance
    drv = *sl::createLidarDriver();

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    sl::IChannel* _channel = (*sl::createSerialPortChannel("/dev/ttyUSB0", 460800));
    if (SL_IS_OK((drv)->connect(_channel))) {
        sl_result op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result)) 
        {
            connectSuccess = true;
        }
        else{
            delete drv;
            drv = NULL;
        }
    }
    if (!connectSuccess) {
        cout << "Failed to connect" << endl;
        return;
    }
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    if (!checkSLAMTECLIDARHealth(drv)) {
        printf("Failed to connect");
        return;
    }

    drv->setMotorSpeed();
    drv->startScan(0,1);

    while (running){
        size_t   count = _countof(nodes);
        op_result = drv->grabScanDataHq(nodes, count);
        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                std::lock_guard<std::mutex> lock(lidarMutex);
                lidarPoints[pos].angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
                lidarPoints[pos].distance = nodes[pos].dist_mm_q2/4.0f;
                lidarPoints[pos].quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT; 
            }
        }
    }
 
    cout << "Stopping..." << endl;
    if (drv) {
        drv->stop();
        delay(200);
        drv->setMotorSpeed(0);
        delete drv;
        drv = NULL;
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
