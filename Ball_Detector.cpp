#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <time.h>
#include <windows.h>

using namespace cv;
using namespace std;
using namespace rs2;

// Prompts the user for the brightness level of the current environment.
int getBrightnessLevel() {
    int BL;
    cout << "Environment Brightness Levels: " << endl;
    cout << "  1 - Dark" << endl;
    cout << "  2 - Average" << endl;
    cout << "  3 - Bright" << endl;
    cout << "Enter Environment Brightness Level: "; cin >> BL;
    return BL;
}

// Gets Y-Dimensional Midpoint.
Point getMidpointY(Point a, Point b) {
    int newY = ((a.y) + (b.y)) / 2;
    return Point(a.x, newY);
}

// Gets X-Dimensional Midpoint.
Point getMidpointX(Point a, Point b) {
    int newX = ((a.x) + (b.x)) / 2;
    return Point(newX, a.y);
}

// Gets the Midpoint of four points.
Point getMidpoint(Point a, Point b, Point c, Point d) {
    int newX = ((a.x) + (b.x)) / 2;
    int newY = ((c.y) + (d.y)) / 2;
    return Point(newX, newY);
}

// Calculates the yaw to the given pixel.
double calculateYaw(int pixelX){
    int centerX = 848 / 2; 
    double horizontalView = 91.2;
	double yaw = ((double)(pixelX - centerX)/(double)centerX)*((double)horizontalView/2.0);
	return round(-yaw);
}

// Tries getting the depth of neighboring points to the centerpoint if the centerpoint does not
// correspond to any depth data.
float getNonZero(float one, float two, float three) {
    if ((one == 0.0f) && (two == 0.0f)) {
        return three;
    } else if ((two == 0.0f) && (three == 0.0f)) {
        return one;
    } else {
        return two;
    }
}

// Applies brightness change, crops ROI, blurs, and filters the ball color HSV for computational efficiency.
Mat preprocessFrame(Mat frame, int imgHeight, int imgWidth, int newHeight, int brightnessLevel) {
    if (brightnessLevel == 1) {
        frame.convertTo(frame, -1, 1, 50);
    } else if (brightnessLevel == 3) {
        frame.convertTo(frame, -1, 1, -50);
    } 
    frame = frame(Rect(0, newHeight, imgWidth, imgHeight - newHeight));
    Mat blurred;
    GaussianBlur(frame, blurred, Size(11, 11), 0, 0);
    Mat hsv;
    cvtColor(blurred, hsv, COLOR_BGR2HSV);
    Mat mask;
    inRange(hsv, Scalar(20, 100, 100), Scalar(30, 255, 255), mask);
    erode(mask, mask, 2);
    dilate(mask, mask, 2);
    return mask;
}

// Gets the leftmost, rightmost, topmost, and bottommost points of the ball.
vector<Point> getBallExtremes(vector<Point> pts) {
    vector<Point> extremeCoordinates;
    Point extLeft  = *min_element(pts.begin(), pts.end(), 
                    [](const Point& lhs, const Point& rhs) {
                        return lhs.x < rhs.x;
                  }); 
    Point extRight = *max_element(pts.begin(), pts.end(),
                    [](const Point& lhs, const Point& rhs) {
                        return lhs.x < rhs.x;
                  });
    Point extTop   = *min_element(pts.begin(), pts.end(), 
                    [](const Point& lhs, const Point& rhs) {
                        return lhs.y < rhs.y;
                  }); 
    Point extBot   = *max_element(pts.begin(), pts.end(),
                    [](const Point& lhs, const Point& rhs) {
                        return lhs.y < rhs.y;
                  });
    extremeCoordinates[0] = extLeft;
    extremeCoordinates[1] = extRight;
    extremeCoordinates[2] = extTop;
    extremeCoordinates[3] = extBot;
    return extremeCoordinates;
}

// Runs calculations on various points of the ball contour to determine if the contour is really just a cluster of balls.
// NOTE: I know the code for this function is terrible. I wrote it in high school. I know better than this. :(
float pointCalculations(rs2::depth_frame depthframe, int newH, Point mpL, Point mpR, Point mpT,  Point mpB, 
                        Point qpR, Point qpR2, Point qpL, Point qpL2, Point qpT2, Point p, float ballD) {

    float d = 0.18f; // Approx. diameter of the ball in meters
    float halfD = 0.09f;
    if ((depthframe.get_distance((int)qpR.x, (int)qpR.y + newH)) > ((depthframe.get_distance((int)qpL.x, (int)qpL.y + newH)) + halfD)) {
        float ballD1 = depthframe.get_distance((int)qpR.x, (int)qpR.y + newH);
        float ballD2 = depthframe.get_distance((int)qpL.x, (int)qpL.y + newH);
        if (ballD1 < ballD2) {
            ballD = ballD1;
        } else {
            ballD = ballD2;
        }
    }
    else if ((depthframe.get_distance((int)mpR.x, (int)mpR.y + newH)) > ((depthframe.get_distance((int)qpL.x, (int)qpL.y + newH)) + halfD)) {
        float ballD1 = depthframe.get_distance((int)mpR.x, (int)mpR.y + newH);
        float ballD2 = depthframe.get_distance((int)qpL.x, (int)qpL.y + newH);
        if (ballD1 < ballD2) {
            ballD = ballD1;
        } else {
            ballD = ballD2;
        }
    }
    else if ((depthframe.get_distance((int)qpR2.x, (int)qpR2.y + newH)) > ((depthframe.get_distance((int)qpL.x, (int)qpL.y + newH)) + halfD)) {
        float ballD1 = depthframe.get_distance((int)qpR2.x, (int)qpR2.y + newH);
        float ballD2 = depthframe.get_distance((int)qpL.x, (int)qpL.y + newH);
        if (ballD1 < ballD2) {
            ballD = ballD1;
        } else {
            ballD = ballD2;
        }
    }
    else if ((depthframe.get_distance((int)qpL.x, (int)qpL.y + newH)) > ((depthframe.get_distance((int)qpR.x, (int)qpR.y + newH)) + halfD)) {
        float ballD1 = depthframe.get_distance((int)qpL.x, (int)qpL.y + newH);
        float ballD2 = depthframe.get_distance((int)qpR.x, (int)qpR.y + newH);
        if (ballD1 < ballD2) {
            ballD = ballD1;
        } else {
            ballD = ballD2;
        }
    }
    else if ((depthframe.get_distance((int)mpL.x, (int)mpL.y + newH)) > ((depthframe.get_distance((int)qpR.x, (int)qpR.y + newH)) + halfD)) {
        float ballD1 = depthframe.get_distance((int)mpL.x, (int)mpL.y + newH);
        float ballD2 = depthframe.get_distance((int)qpR.x, (int)qpR.y + newH);
        if (ballD1 < ballD2) {
            ballD = ballD1;
        } else {
            ballD = ballD2;
        }
    }
    else if ((depthframe.get_distance((int)qpL2.x, (int)qpL2.y + newH)) > ((depthframe.get_distance((int)qpR.x, (int)qpR.y + newH)) + halfD)) {
        float ballD1 = depthframe.get_distance((int)qpL2.x, (int)qpL2.y + newH);
        float ballD2 = depthframe.get_distance((int)qpR.x, (int)qpR.y + newH);
        if (ballD1 < ballD2) {
            ballD = ballD1;
        } else {
            ballD = ballD2;
        }
    } 
    else {
        if (((depthframe.get_distance((int)qpT2.x, (int)qpT2.y + newH)) > ((depthframe.get_distance((int)mpR.x, (int)mpR.y + newH)) + d))) {
            ballD = depthframe.get_distance((int)mpR.x, (int)mpR.y + newH);
        }
        ballD = getNonZero(depthframe.get_distance((int)p.x, (int)p.y + newH), depthframe.get_distance((int)mpL.x, (int)mpL.y + newH), depthframe.get_distance((int)mpR.x, (int)mpR.y + newH));
    }
    return ballD;
}

int main() {
    
    // Initialize Realsense stream
    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config cfg;
    int w = 848;
    int h = 480;
    cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, 30);
    pipe.start(cfg);

    int BL = getBrightnessLevel();

    while (true) {

        // Get frames from the Realsense camera
        rs2::frameset data = pipe.wait_for_frames(); 
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned_frames = align.process(data);
        rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame depthframe = aligned_frames.get_depth_frame();
        
        // Get the image Mat from the color frame for CV
        Mat frame(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        // Preprocess the frame for computational efficiency
        int newH = h * 0.45;
        Mat mask = preprocessFrame(frame, h, w, newH, BL);

        // Get the contours of the image
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) continue;
        
        // Drawing is for calculations, Display is for debugging/viewing.
        Mat drawing = Mat::zeros(mask.size(), CV_8U);
        Mat display = Mat::zeros(mask.size(), CV_8U);
        Scalar color = Scalar(0, 0, 255);

        // These variables represent various details about the closest ball to the robot.
        double largestCont = 0.0;
        int largestContInd;
        Point largestContPoint;
        float closestBallDist = 0.0f;
        float smallestD = 100.0f;
        float ballD = 100.0f;

        // Determine if each contour is a ball
        for(size_t i = 0; i< contours.size(); i++) {
            if (contourArea(contours[i]) <= 20.0) continue; // No ball contour should be smaller than __ pixels.
        
            vector<vector<Point> > contours_poly(1);
            vector<Point2f>center(1);
            vector<float>radius(1);

            vector<Point> ballContour = contours[i];
            Rect r = boundingRect(ballContour);

            vector<Moments> mu(1);
            mu[0] = moments(ballContour, true);
            Moments M = mu[0];
            minEnclosingCircle( ballContour, center[0], radius[0] );
            circle( display, center[0], (int)radius[0], 255, 1, 8, 0 ); 
            circle( drawing, center[0], (int)radius[0], 255, 1, 8, 0 ); 

            vector<Point> pts;
            Mat output;
            findNonZero(drawing, output);
            for (int i = 0; i < output.total(); i++) {
                pts.push_back(Point(output.at<Point>(i).x, output.at<Point>(i).y));
            }

            Point extLeft  = getBallExtremes(pts)[0];
            Point extRight = getBallExtremes(pts)[1];
            Point extTop   = getBallExtremes(pts)[2];
            Point extBot   = getBallExtremes(pts)[3];

            Point p = getMidpoint(extLeft, extRight, extTop, extBot); // Midpoint of the contour
            Point mpL = getMidpointX(p, extLeft);
            Point mpR = getMidpointX(p, extRight);
            float dist = getNonZero(depthframe.get_distance((int)p.x, (int)p.y + newH), depthframe.get_distance((int)mpL.x, (int)mpL.y + newH), depthframe.get_distance((int)mpR.x, (int)mpR.y + newH));
            cout << abs(extTop.y - extBot.y) * (dist) << endl;

            // Thresholds circle size by the product: (ball diameter in pixels) * (Distance of the ball).
            if ((abs((extTop.y - extBot.y) * (dist)) > 320) ||
                ((abs(extTop.y - extBot.y) * (dist)) < 90)) {
                    drawing = Mat::zeros(mask.size(), CV_8U);
                    continue;
            }
            // If the circle is an extreme oval for WHATEVER reason, skip. 
            if ((abs(extTop.y - extBot.y) > (abs(extRight.x - extLeft.x) * 2.5)) || (abs(extRight.x - extLeft.x) > (abs(extTop.y - extBot.y) * 2.5))) {
                drawing = Mat::zeros(mask.size(), CV_8U);
                continue;
            }

            // If the product indicates that balls may be next to each other, run point calculations.
            if ((abs((extTop.y - extBot.y) * (dist)) > 160)) {
                cout << "Marked for review" << endl;
                circle(display, getMidpointY(p, extTop), 1, 155, -1);
                circle(display, getMidpointY(p, extBot), 1, 155, -1);
                circle(display, getMidpointX(p, extLeft), 1, 155, -1);
                circle(display, getMidpointX(p, extRight), 1, 155, -1);
                circle(display, p, 5, Scalar(100), -1);

                Point mpT = getMidpointY(p, extTop);
                Point mpB = getMidpointY(p, extBot);
                Point qpT2 = getMidpointY(mpT, extTop);
                circle(display, qpT2, 1, 155, -1);
                Point qpR = getMidpointX(p, mpR);
                circle(display, qpR, 1, 155, -1);
                Point qpR2 = getMidpointX(mpR, extRight);
                circle(display, qpR2, 1, 155, -1);
                Point qpL = getMidpointX(p, mpL);
                circle(display, qpL, 1, 155, -1);
                Point qpL2 = getMidpointX(mpL, extLeft);
                circle(display, qpL2, 1, 155, -1);

                ballD = pointCalculations(depthframe, newH, mpL, mpR, mpT, mpB, qpR, qpR2, qpL, qpL2, qpT2, p, ballD);
            } 
            else {
                ballD = getNonZero(depthframe.get_distance((int)p.x, (int)p.y + newH), depthframe.get_distance((int)mpL.x, (int)mpL.y + newH), depthframe.get_distance((int)mpR.x, (int)mpR.y + newH));
                cout << "Product Singular" << endl;
            }
            cout << "Difference: " << abs(extTop.y - extBot.y) << " Distance: " << dist << endl;
            cout << abs(extTop.y - extBot.y) * (dist) << endl;
            
            // Update variables representing the closest ball in frame.
            if ((smallestD > ballD) && (ballD != 0.0f)) {
                largestContInd = (int)i;
                largestContPoint = p;
                closestBallDist = dist;
                smallestD = ballD;
            }
            drawing = Mat::zeros(mask.size(), CV_8U); 
        } 
        circle(display, largestContPoint, 5, Scalar(255), -1);
        cout << "Distance to closest ball: " << smallestD << " Angle to closest ball: " << calculateYaw(largestContPoint.x) << endl;
        
        imshow("Contours", display);
        imshow("Feed", frame);
        if (waitKey(1) >= 0) {
            break;
        }
    } 
    return 0;
}