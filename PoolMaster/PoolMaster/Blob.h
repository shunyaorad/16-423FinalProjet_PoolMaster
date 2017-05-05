//  Background Subtraction and Tracking
//  Blob.h
//  Reference:
//  https://github.com/MicrocontrollersAndMore/OpenCV_3_Multiple_Object_Tracking_by_Image_Subtraction_Cpp
//

#ifndef MY_BLOB
#define MY_BLOB

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

///////////////////////////////////////////////////////////////////////////////////////////////////
class Blob {
public:
    // member variables ///////////////////////////////////////////////////////////////////////////
    std::vector<cv::Point> currentContour;
    
    cv::Rect currentBoundingRect;
    
    std::vector<cv::Point> centerPositions;
    
    std::vector<cv::Point2f> trajectoryPoints; // keep track of points for trajectory.
    
    double dblCurrentDiagonalSize;
    double dblCurrentAspectRatio;
    
    bool blnCurrentMatchFoundOrNewBlob;
    
    bool blnStillBeingTracked;
    
    int intNumOfConsecutiveFramesWithoutAMatch;
    
    int ID;
    
    cv::Point predictedNextPosition;
    
    // function prototypes ////////////////////////////////////////////////////////////////////////
    Blob(std::vector<cv::Point> _contour);
    void predictNextPosition(void);
    
};

#endif 


