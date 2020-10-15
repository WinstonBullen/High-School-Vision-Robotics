#include <nospoon/camera.h>
using namespace std;
using namespace cv;
using namespace picojson;

// 60% OF CODE OMITTED FOR PRIVACY

//One time setup
bool Camera::setup(string path) {
    cout << "Load Settings!" << endl;
    if(!loadCameraSettings(path)) {
        cerr << "Error loading settings!" << endl;
        return false;
    }
    return true;
}

Camera* CameraUnit::getCamera(CameraType ctype) {
    //greedy!  Gets the first instance of a camera
    for(int i = 0; i < cameras.size(); i++) {
        if (cameras.at(i).getCurrentSettings().camType == ctype) {
            return &cameras.at(i);
        }
    }
}

Camera* CameraUnit::getCamera(CameraType ctype, int cNum) {    
    int camCount = 1;
    int lastInd = -1;
    for(int i = 0; i < cameras.size(); i++) {
        if (cameras.at(i).getCurrentSettings().camType == ctype) {
            lastInd = i;
            if (camCount < cNum) camCount++;
            else return &cameras.at(i);
        } 
    }
    return &cameras.at(lastInd);
}


CamMat* CameraUnit::getFrames() {
    return frames[framePointer];
}


CamMat* CameraUnit::getFrame(CameraType ctype) {
    for(int i = 0; i<cameras.size(); i++) {
        if(frames[framePointer][i].camType == ctype)
        {
            //This is greedy - just return the first IR.
            return &frames[framePointer][i];
        }
    }
}

CamMat* CameraUnit::getFrame(CameraType ctype, int cNum) {
    int camCounter = 1;
    int lastInd = -1;
    for(int i = 0; i<cameras.size(); i++) {
        if(frames[framePointer][i].camType == ctype)
        {
            lastInd = i;
            if (camCounter < cNum) camCounter++;
            else return &frames[framePointer][i];
        }
    }
    return &frames[framePointer][lastInd];
}
