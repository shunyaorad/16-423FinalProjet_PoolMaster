//
//  ViewController.m
//  Final Project. Pool Table Master.
//
//  Created by SHUN YAO on 4/10/17.
//  Copyright © 2017 CMU. All rights reserved.
//


#import "ViewController.h"

#ifdef __cplusplus
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/video/background_segm.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <unordered_map>
#include "Blob.h"
#endif

//======= Variables Declaration =================

// define colors
cv::Scalar RED      = cv::Scalar(255,  0,      0);
cv::Scalar GREEN    = cv::Scalar(0,    255,    0);
cv::Scalar BLUE     = cv::Scalar(0,    0,      255);
cv::Scalar WHITE    = cv::Scalar(255,  255,    255);
cv::Scalar BLACK    = cv::Scalar(0,    0,      0);
cv::Scalar YELLOW   = cv::Scalar(255,  255,    0);
cv::Scalar CYAN     = cv::Scalar(64,   224,    208);
cv::Scalar CRIMSON  = cv::Scalar(220,  20,     60); // Go TARTANS!
cv::Scalar DBLUE    = cv::Scalar(55,   93,     150);
cv::Scalar COLORS[5] {CRIMSON, CYAN, YELLOW, GREEN, DBLUE};

// camera and view constants
float   cam_width   = 1280;
float   cam_height  = 720;
float   view_width;
float   view_height;
float   offset;

// thresholds
int     differenceThreshold = 80;   // thresh for background subtraction
double  minDistThresh       = 10;    // min jump to record  the trajectory points
double  maxDistThresh       = 200;   // max jump allowed for the trajectory points

// virtual table parameters
// switched widht and height for landscape view
//int tableWidth      =   290*2;
//int tableHeight     =   163*2;
int tableWidth      =   286*2;  // working
int tableHeight     =   145.5*2;    // working 161

int homographyWidth  = 82*2;

int background_table_offsetX;
int background_table_offsetY;
//int virtualBallRadius   =   20; // working. should be in scale with the real ball
int virtualBallRadius = 15;

int virtualCueLength    =   100;

// flags for each state
BOOL trackingStarted    = false;
BOOL pointsSelected     = false;
BOOL showConvexHull     = false;
BOOL showPrediction     = false;
BOOL predictionMode     = false;
BOOL backgroundSelected = false;
BOOL trackingFirstTime  = true;
BOOL blnFirstFrame      = true; // for blob detection and tracking
BOOL showBirdView       = false;
BOOL highResolution     = true; // switch between 640x480 and 1280x720
BOOL birdViewFirstTime  = true;
BOOL predictFirstTime   = true;

BOOL selectCueBall      = false;
BOOL selectTargetBall   = false;
BOOL selectGoal         = false;
BOOL cueBallSelected    = false;
BOOL targetBallSelected = false;
BOOL goalPositionSelected = false;
BOOL alignmentMode      = false;


// Mat declarations
cv::Mat currentFrame;
cv::Mat previousFrame;
cv::Mat background;
cv::Mat backgroundForVirtualTable;
cv::Mat mask; // ignore difference outside of the ROI
// virtual pool table
cv::Mat poolTable;
cv::Mat homographyForward;  // warp from ROI image to birdview
cv::Mat homographyBackward; // warp from birdview to ROI

// four points to be matched for virtual table
std::vector<cv::Point2f> pts_dst;
std::vector<cv::Point2f> FourPoints;
std::vector<std::vector<cv::Point2f>> transformed_trajectories;   // trajectories of balls in birdview

std::vector<Blob> blobs;    // for blob detection and tracking
std::unordered_map<int, BOOL> trackedIDs; // true if the blob with that ID is used
std::vector<Blob> trackedBlobs;

cv::Point2f cueBall;
cv::Point2f targetBall;
cv::Point2f goalPosition;

std::vector<cv::Point2f> circle_coords;         // coordinates of balls in warped ROI
std::vector<cv::Point2f> transformed_circles;   // coordinates of balls in birdview
std::vector<cv::Point2f> virtualCuePositions;   // start and end points of virtual cue
std::vector<cv::Point2f> transformed_cue;       // cue positions in warped ROI

//=============================================================

@interface ViewController ()
{
    
    __weak IBOutlet UIView *Picker1;
    __weak IBOutlet UIView *Picker2;
    __weak IBOutlet UIView *Picker3;
    __weak IBOutlet UIView *Picker4;
    
    UITextView *fpsView_;   // Display the current FPS
    int64 curr_time_;       // Store the current time for FPS
    
    __weak IBOutlet UIButton *GetBackgroundButton;
    __weak IBOutlet UIButton *MakePredictionButton;
    
    __weak IBOutlet UIButton *GetConvexHullButton;
    __weak IBOutlet UIButton *StartTrackingButton;
    __weak IBOutlet UIButton *ShowBirdViewButton;
    
    __weak IBOutlet UIButton *PickCueBallButton;
    __weak IBOutlet UIButton *PickTargetBallButton;
    __weak IBOutlet UIButton *PickGoalPositionButton;
    __weak IBOutlet UIButton *AlignCueButton;
    
    
}

@end

@implementation ViewController


@synthesize myImageView;
@synthesize videoCamera;

- (void)viewDidLoad {
    [super viewDidLoad];
    
    NSNumber *value = [NSNumber numberWithInt:UIInterfaceOrientationLandscapeRight];
    [[UIDevice currentDevice] setValue:value forKey:@"orientation"];
    
    if (!highResolution)
    {
        cam_width   = 640;
        cam_height  = 480;
    }
    
    /*
     * since cam width is larger than cam height, we fit the view height to the 
     * screen height and then fit (shorten) the width.
     */
    view_height  =   self.view.frame.size.height;
    view_width =   (int)(cam_width*self.view.frame.size.height / cam_height);
    myImageView = [[UIImageView alloc]
                   initWithFrame:CGRectMake(0.0, 0.0, view_width, view_height)];
    
    // add subview to display
    [self.view addSubview:myImageView];
    [self.view addSubview:MakePredictionButton];
    [self.view addSubview:GetConvexHullButton];
    [self.view addSubview:GetBackgroundButton];
    [self.view addSubview:StartTrackingButton];
    [self.view addSubview:ShowBirdViewButton];
    
    [self.view addSubview:PickCueBallButton];
    [self.view addSubview:PickTargetBallButton];
    [self.view addSubview:PickGoalPositionButton];
    [self.view addSubview:AlignCueButton];
    
    MakePredictionButton.hidden     =   YES;
    ShowBirdViewButton.hidden       =   YES;
    StartTrackingButton.hidden      =   YES;
    PickCueBallButton.hidden        =   YES;
    PickTargetBallButton.hidden     =   YES;
    PickGoalPositionButton.hidden   =   YES;
    GetBackgroundButton.hidden      =   YES;
    AlignCueButton.hidden           =   YES;
    
    // add pickers to the subview to display
    [self.view addSubview:Picker1];
    [self.view addSubview:Picker2];
    [self.view addSubview:Picker3];
    [self.view addSubview:Picker4];
    
    // show pickers
    [self->Picker1 setHidden:NO];
    [self->Picker2 setHidden:NO];
    [self->Picker3 setHidden:NO];
    [self->Picker4 setHidden:NO];
    
    // add gesture for pickers
    [self addGestureRecognizersForViews];
    
    // Initialize the camera
    self.videoCamera = [[MyCvVideoCamera alloc]
                        initWithParentView:myImageView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition =
    AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureVideoOrientation =
    AVCaptureVideoOrientationLandscapeRight;
    self.videoCamera.defaultFPS = 30;
    self.videoCamera.rotateVideo = YES;
    self.videoCamera.defaultAVCaptureSessionPreset =
    highResolution ? AVCaptureSessionPreset1280x720 : AVCaptureSessionPreset640x480;
    
    // Add the FPS text to the view
    fpsView_ = [[UITextView alloc] initWithFrame:CGRectMake(0,15,view_width,50)];
    [fpsView_ setOpaque:false]; // Set to be Opaque
    [fpsView_ setBackgroundColor:[UIColor clearColor]]; // Set background color to be clear
    [fpsView_ setTextColor:[UIColor redColor]]; // Set text to be RED
    [fpsView_ setFont:[UIFont systemFontOfSize:30]]; // Set the Font size
    [self.view addSubview:fpsView_];
    
    // Set up large background for pool table to be laid on
    background  =   cv::Mat(cam_height, cam_width, CV_8UC3, cv::Scalar(0,0,0));
    backgroundForVirtualTable   =   background.clone();
    poolTable   =   cv::Mat(tableHeight, tableWidth, CV_8UC3, cv::Scalar(59,174,59));
    
    // for virtual table homography
    pts_dst.push_back(cv::Point2f(0, 0));
    pts_dst.push_back(cv::Point2f(homographyWidth, 0));
    pts_dst.push_back(cv::Point2f(homographyWidth, tableHeight));
    pts_dst.push_back(cv::Point2f(0, tableHeight));
    
    
    // offsets are used to put ROI and pool table in middle of iPad screen
    //background_table_offsetX = (cam_width - tableWidth)/2;
    //background_table_offsetY = (cam_height - tableHeight)/2;
    
    // offsets are used to put pool table in middle of iPad screen
    background_table_offsetX = (view_width - tableWidth)/4; // not sure why /2 doesnt work
    background_table_offsetY = (view_height - tableHeight)/2;
    
    // start video
    [self.videoCamera start];
}

- (IBAction)GetBackgroundButtonPressed:(id)sender {
    
    GetBackgroundButton.hidden  = YES;
    ShowBirdViewButton.hidden   = YES;
    StartTrackingButton.hidden  = NO;
    GetConvexHullButton.hidden  = NO;
    StartTrackingButton.hidden  = YES;
    MakePredictionButton.hidden = YES;

    
    PickCueBallButton.hidden        =   YES;
    PickTargetBallButton.hidden     =   YES;
    PickGoalPositionButton.hidden   =   YES;
    alignmentMode                   =   false;
    AlignCueButton.hidden           =   YES;
    
    [self.videoCamera unlockFocus];
    
    [self->Picker1 setHidden:NO];
    [self->Picker2 setHidden:NO];
    [self->Picker3 setHidden:NO];
    [self->Picker4 setHidden:NO];
    
    clearTrackedRecords();
    poolTable   =   cv::Mat(tableHeight, tableWidth, CV_8UC3, cv::Scalar(59,174,59));
    
    trackingFirstTime   = true;
    backgroundSelected  = false;
    pointsSelected      = false;
    showConvexHull      = false;
    trackingStarted     = false;
    showBirdView        = false;
    predictionMode     = false;
}

- (IBAction)GetConvexButtonPressed:(id)sender {
    
    FourPoints = [self getFourPoints];
    homographyForward   = cv::findHomography(FourPoints, pts_dst);
    homographyBackward  = cv::findHomography(pts_dst, FourPoints);

    ShowBirdViewButton.hidden   = YES;
    StartTrackingButton.hidden  = NO;
    MakePredictionButton.hidden = NO;
    GetConvexHullButton.hidden  = YES;
    StartTrackingButton.hidden  = NO;
    
    GetBackgroundButton.hidden      =   NO;
    PickCueBallButton.hidden        =   YES;
    PickTargetBallButton.hidden     =   YES;
    PickGoalPositionButton.hidden   =   YES;
    alignmentMode                   =   false;
    AlignCueButton.hidden           =   YES;
    
    [self->Picker1 setHidden:YES];
    [self->Picker2 setHidden:YES];
    [self->Picker3 setHidden:YES];
    [self->Picker4 setHidden:YES];
    
    clearTrackedRecords();
    poolTable   =   cv::Mat(tableHeight, tableWidth, CV_8UC3, cv::Scalar(59,174,59));
    
    pointsSelected          = true;
    trackingFirstTime       = true;
    showConvexHull          = true;
    trackingStarted         = false;
    showBirdView            = false;
    predictionMode          = false;
}
- (IBAction)MakePredictionButtonPressed:(id)sender {
    predictionMode = true;
    blnFirstFrame = true;
    PickCueBallButton.hidden        =   NO;
    PickTargetBallButton.hidden     =   NO;
    PickGoalPositionButton.hidden   =   NO;
    GetBackgroundButton.hidden      =   NO;
    GetConvexHullButton.hidden      =   NO;
    MakePredictionButton.hidden     =   YES;
    alignmentMode                   =   false;
    AlignCueButton.hidden           =   YES;
    clearTrackedRecords();
}

- (IBAction)PickCueButtonPressed:(id)sender {
    selectCueBall       = true;
    selectTargetBall    = false;
    selectGoal          = false;
}

- (IBAction)PickTargetButtonPressed:(id)sender {
    selectCueBall       = false;
    selectTargetBall    = true;
    selectGoal          = false;
}

- (IBAction)PickGoalButtonPressed:(id)sender {
    selectCueBall       = false;
    selectTargetBall    = false;
    selectGoal          = true;
}

- (IBAction)AlignCueButtonPressed:(id)sender {
    cueBallSelected         = false;
    targetBallSelected      = false;
    goalPositionSelected    = false;
    
    AlignCueButton.hidden   =   YES;
    alignmentMode           =   true;
    predictionMode          =   false;
    if (virtualCuePositions.size() != 0)
    {
        cv::perspectiveTransform( virtualCuePositions, transformed_cue, homographyBackward);
    }
    
}

- (IBAction)StartTrackingButtonPressed:(id)sender {
    trackingStarted             = true;
    showBirdView                = false;
    ShowBirdViewButton.hidden   = NO;
    StartTrackingButton.hidden  = YES;
    MakePredictionButton.hidden = YES;
    predictionMode              = NO;
    predictFirstTime            = true;
    GetBackgroundButton.hidden  =   NO;
    GetConvexHullButton.hidden  =   NO;
    PickCueBallButton.hidden        =   YES;
    PickTargetBallButton.hidden     =   YES;
    PickGoalPositionButton.hidden   =   YES;
    alignmentMode                   =   false;
    AlignCueButton.hidden           =   YES;
    
    [self.videoCamera lockFocus];
}

- (IBAction)ShowBirdViewButtonPressed:(id)sender {
    
    poolTable   =   cv::Mat(tableHeight, tableWidth, CV_8UC3, cv::Scalar(59,174,59));
    
    StartTrackingButton.hidden  = YES;
    ShowBirdViewButton.hidden   = YES;
    MakePredictionButton.hidden = YES;
    GetConvexHullButton.hidden  = NO;
    
    PickCueBallButton.hidden        =   YES;
    PickTargetBallButton.hidden     =   YES;
    PickGoalPositionButton.hidden   =   YES;
    
    showBirdView        =   true;
    trackingStarted     =   false;
    showConvexHull      =   false;
    trackingFirstTime   =   true;
    birdViewFirstTime   =   true;
    alignmentMode                   =   false;
    AlignCueButton.hidden           =   YES;
}

// initialize the interactive pickers
-(void)addGestureRecognizersForViews
{
    UIPanGestureRecognizer *pan=[[UIPanGestureRecognizer alloc]
                                  initWithTarget:self action:@selector(panEventHandler:)];
    [self->Picker1 addGestureRecognizer:pan];
    
    pan=[[UIPanGestureRecognizer alloc]
         initWithTarget:self action:@selector(panEventHandler:)];
    [self->Picker2 addGestureRecognizer:pan];
    
    pan=[[UIPanGestureRecognizer alloc]
         initWithTarget:self action:@selector(panEventHandler:)];
    [self->Picker3 addGestureRecognizer:pan];
    
    pan=[[UIPanGestureRecognizer alloc]
         initWithTarget:self action:@selector(panEventHandler:)];
    [self->Picker4 addGestureRecognizer:pan];
}

// moving event for interactive picker
-(void)panEventHandler:(UIPanGestureRecognizer *)pan
{
    [self translateView:pan.view becauseOfGestureRecognizer:pan];
}

// moving event for interactive picker
-(void)translateView:(UIView *)view becauseOfGestureRecognizer:(UIPanGestureRecognizer *)pan
{
    UIView * target     =   pan.view;
    CGPoint translation =   [pan translationInView:self.view];
    target.center       =   CGPointMake(target.center.x + translation.x,
                                        target.center.y + translation.y);
    [pan setTranslation:CGPointZero inView:self.view];
}

//The touch event handling method
- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch = [touches anyObject];
    // Get the specific point that was touched
    CGPoint point = [touch locationInView:self.view];
    
    float xTouched = point.x * cam_width    / view_width - background_table_offsetX;
    float yTouched = point.y * cam_height   / view_height - background_table_offsetY;
    //NSLog(@"Touched: (%f, %f)\n", xTouched, yTouched);
    
    if (predictionMode && yTouched >= 0 && yTouched <= cam_height
        && xTouched >= 0 && xTouched < cam_width)
    {
        cv::Point2f pointSelected(xTouched, yTouched);
        
        // Goal position can be anywhere on the pool table touched
        if (selectGoal && !goalPositionSelected)
        {
            goalPosition            = pointSelected;
            goalPositionSelected     = true;
            cv::circle( poolTable, pointSelected, virtualBallRadius, CRIMSON, 3, CV_AA, 0 );
            cv::circle( poolTable, pointSelected, 3, BLACK, 3, -1, 0 ); // ball center
        }
        
        for (size_t i = 0; i < transformed_circles.size(); i++ )
        {
            auto currentCircle  = transformed_circles[i];
            float distance = distanceBetweenPoints(currentCircle, pointSelected);
            if (distance > 30)
            {
                // printf("No ball selected\n");
                continue;
            }
            else if (distance < 30)
            {
                // printf("Found circle selected\n");
                
                if (selectCueBall && !cueBallSelected)
                {
                    cueBall = transformed_circles[i];
                    cueBallSelected = true;
                    cv::circle( poolTable, currentCircle, virtualBallRadius+10, CYAN, 3, CV_AA, 0 );
                    cv::circle( poolTable, currentCircle, 3, BLACK, 3, -1, 0 ); // ball center
                }
                if (selectTargetBall && !targetBallSelected)
                {
                    targetBall = transformed_circles[i];
                    targetBallSelected = true;
                    cv::circle( poolTable, currentCircle, virtualBallRadius+10, YELLOW, 3, CV_AA, 0 );
                    cv::circle( poolTable, currentCircle, 3, BLACK, 3, -1, 0 ); // ball center
                }
            }
        }
    }
    
    /* turn on alignment mode button */
    if (cueBallSelected && targetBallSelected && goalPositionSelected)
    {
        PickCueBallButton.hidden        =   YES;
        PickTargetBallButton.hidden     =   YES;
        PickGoalPositionButton.hidden   =   YES;
        AlignCueButton.hidden           =   NO;
    }
    /////////////////////////////////////////////////
    
}



// Process Video image here ======================================
- (void)processImage:(cv::Mat&)image
{
    // Estimate the frames per second (FPS) =======================
    int64 next_time = cv::getTickCount(); // Get the next time stamp
    float fps = (float)cv::getTickFrequency()/(next_time - curr_time_); // Estimate the fps
    curr_time_ = next_time; // Update the time
    NSString *fps_NSStr = [NSString stringWithFormat:@"FPS = %2.2f",fps];
    
    // Have to do this so as to communicate with the main thread
    // to update the text display
    dispatch_sync(dispatch_get_main_queue(), ^{
        fpsView_.text = fps_NSStr;
    });
    
    //==============================================================
    
    cvtColor(image, image, CV_RGBA2BGR);
    cv::Mat displayImg = image.clone();  // make a clean copy of image to be displayed
    
    
    if (!pointsSelected)
    {
        
        FourPoints = [self getFourPoints];
        for (auto point : FourPoints)
        {
            PutCoordinateText(image, point);
        }
    }
    
    // Crop image of the selected ROI
    if (pointsSelected & !backgroundSelected) // Get Difference button pressed
    {
        std::vector<cv::Point2f> ROI_Poly; // store ROI points
        // create a polygon from vertices
        cv::approxPolyDP(FourPoints, ROI_Poly, 1, true);
        std::vector<cv::Point> ROI_Poly_int = floatPoint2Int(ROI_Poly);
        // create mask
        mask = cv::Mat::zeros(image.size(), image.type());
        // fill polygon white
        cv::fillConvexPoly(mask, &ROI_Poly_int[0], 4, WHITE, CV_AA, 0);
        // Create background image
        background = cv::Mat::zeros(image.size(), image.type());
        image.copyTo(background, mask);
        cv::cvtColor(background, background, CV_BGR2GRAY);
        // Turn mask into 8UC1 for later use during get difference
        cv::cvtColor(mask, mask, CV_BGR2GRAY);
        backgroundSelected = true;
    }
    
    if(backgroundSelected && !showBirdView && predictFirstTime)
    {
        image.copyTo(currentFrame, mask); // mask so that only consider ROI region
        //currentFrame = image.clone();
        cv::Mat diffWithBackground;
        cv::Mat diffWithBackgroundThresh;
        std::vector<Blob> currentFrameBlobs;
        cv::cvtColor(currentFrame, currentFrame, CV_BGR2GRAY);
        //cv::GaussianBlur(currentFrame, currentFrame, cv::Size(5, 5), 0);
        //cv::boxFilter(currentFrame, currentFrame, -1, cv::Size(5,5));
        cv::absdiff(currentFrame, background, diffWithBackground);
        cv::threshold(diffWithBackground, diffWithBackgroundThresh, differenceThreshold, 255.0, CV_THRESH_BINARY);
        if (showConvexHull)
        {
            // Remove noise by dilation and erosion. This causes slowdown in FPS.
            cv::Mat structuringElement3x3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::dilate(diffWithBackgroundThresh, diffWithBackgroundThresh, structuringElement3x3);
            cv::dilate(diffWithBackgroundThresh, diffWithBackgroundThresh, structuringElement3x3);
            cv::erode(diffWithBackgroundThresh, diffWithBackgroundThresh, structuringElement3x3);
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(diffWithBackgroundThresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::Mat imgContours(diffWithBackgroundThresh.size(), CV_8UC3, BLACK);
            cv::drawContours(imgContours, contours, -1, WHITE, -1);
            std::vector<std::vector<cv::Point> > convexHulls(contours.size());
            for (unsigned int i = 0; i < contours.size(); i++)
            {
                cv::convexHull(contours[i], convexHulls[i]);
            }
            
            // Find blobs that we care about
            for (auto &convexHull : convexHulls) {
                Blob possibleBlob(convexHull);
                if (possibleBlob.currentBoundingRect.area() > 200 &&
                    possibleBlob.currentBoundingRect.area() < 10000 &&
                    possibleBlob.dblCurrentAspectRatio >= 0.5 && // aspect ratio of ball should be ~1.0
                    possibleBlob.dblCurrentAspectRatio <= 1.5 &&
                    possibleBlob.currentBoundingRect.width > 15 &&
                    possibleBlob.currentBoundingRect.height > 15 &&
                    possibleBlob.dblCurrentDiagonalSize > 30 &&
                    (cv::contourArea(possibleBlob.currentContour) / (double)possibleBlob.currentBoundingRect.area()) > 0.40) {
                    currentFrameBlobs.push_back(possibleBlob);
                }
            }
            if (!trackingStarted && blnFirstFrame == true)
            {
                for (auto &currentFrameBlob : currentFrameBlobs)
                {
                    blobs.push_back(currentFrameBlob);
                }
                blnFirstFrame = false;
            }
            
            // update blobs if its not prediction mode
            else if (!trackingStarted)
            {
                matchCurrentFrameBlobsToExistingBlobs(blobs, currentFrameBlobs);
            }
            
            cv::Mat imgConvexHulls(diffWithBackgroundThresh.size(), CV_8UC3, BLACK);
            cv::drawContours(imgConvexHulls, convexHulls, -1, WHITE, -1);
            image = cv::Mat::zeros(image.size(), image.type());
            imgConvexHulls.copyTo(image, mask);
            if(!trackingStarted)
            {
                drawBlobInfoOnImage(blobs, image);
            }
            if (trackingStarted)
            {
                if (trackingFirstTime)
                {
                    trackingFirstTime = false;
                    int ID = 0;
                    for (auto blob : currentFrameBlobs)
                    {
                        blob.trajectoryPoints.push_back(cv::Point2f(blob.centerPositions.back().x,
                                                                    blob.centerPositions.back().y));
                        trackedBlobs.push_back(blob);
                        trackedBlobs[ID].ID = ID;
                        ID++;
                    }
                }
                matchCurrentFrameBlobsToTrackedBlobs(trackedBlobs, currentFrameBlobs, trackedIDs);
                
                image = displayImg.clone();
                drawBlobInfoOnImage(trackedBlobs, image);
                drawTrajectoryOnImage(trackedBlobs, image);
                
            }
            // prepare for next frame
            currentFrameBlobs.clear();
        }
    }
    
    if (predictionMode)
    {
        // show birdview and predict cue position
        // do not update blob positions once entered prediction mode
        if (predictFirstTime)
        {
            extractBlobPositionsToVector(blobs, circle_coords);
            if (circle_coords.size() != 0)
            {
                predictFirstTime = false;
                cv::perspectiveTransform( circle_coords, transformed_circles, homographyForward);
                int colorID = 0;
                for (auto circle : transformed_circles )
                {
                    cv::circle( poolTable, circle, virtualBallRadius,COLORS[colorID%5], -1, CV_AA, 0 ); // virtual ball
                    cv::circle( poolTable, circle, 3, BLACK, 3, -1, 0 ); // ball center
                    
                    PutCoordinateText(poolTable, circle);
                }
            }
        }
        
        // Connect cue, target, and goal, shifted target
        if (cueBallSelected && targetBallSelected && goalPositionSelected)
        {
            cv::Point2f shift_target = FindShiftAmount(goalPosition, targetBall, virtualBallRadius);
            cv::Point2f shiftedTarget = targetBall + shift_target;
            
            cv::Point2f shift_cue_end = FindShiftAmount(shiftedTarget, cueBall, virtualBallRadius+virtualCueLength);
            cv::Point2f shiftedCue1 = cueBall + shift_cue_end;
            
            cv::Point2f shift_cue_start = FindShiftAmount(shiftedTarget, cueBall, virtualBallRadius-10);
            cv::Point2f shiftedCue2 = cueBall + shift_cue_start;
            
            virtualCuePositions.push_back(shiftedCue1);
            virtualCuePositions.push_back(shiftedCue2);
            
            cv::circle(poolTable, shiftedTarget, virtualBallRadius, CYAN, 3, CV_AA, 0);
            
            cv::arrowedLine(poolTable, cueBall, shiftedTarget, DBLUE, 2, CV_AA);
            cv::arrowedLine(poolTable, targetBall, goalPosition, DBLUE, 2, CV_AA);
            cv::arrowedLine(poolTable, shiftedCue1, shiftedCue2, DBLUE, 2, CV_AA);
        }
        
        backgroundForVirtualTable = cv::Mat(cam_height, cam_width, CV_8UC3, cv::Scalar(0,0,0));
        poolTable.copyTo(backgroundForVirtualTable(cv::Rect(background_table_offsetX,
                                                            background_table_offsetY,
                                                            poolTable.cols,
                                                            poolTable.rows)));
        image = backgroundForVirtualTable.clone();
    }
    
    if (alignmentMode)
    {
        if (transformed_cue.size() != 0)
        {
            cv::arrowedLine(image, transformed_cue[0], transformed_cue[1], CRIMSON, 3, CV_AA);
        }
    }
    
    if(showBirdView)
    {
        // only draw once to save computation
        if (birdViewFirstTime)
        {
            birdViewFirstTime = false;
            warpBlobTrajectories(trackedBlobs, transformed_trajectories, homographyForward);
            drawTrajectoriesOnImage(transformed_trajectories, poolTable);
            // put pool table on top of black background
            backgroundForVirtualTable = cv::Mat(cam_height, cam_width, CV_8UC3, cv::Scalar(0,0,0));
            poolTable.copyTo(backgroundForVirtualTable(cv::Rect(background_table_offsetX,
                                                 background_table_offsetY,
                                                 poolTable.cols,
                                                 poolTable.rows)));
        }
        image = backgroundForVirtualTable.clone();
    }
}

- (void)viewDidDisappear:(BOOL)animated
{
    [super viewDidDisappear:animated];
    if (isCapturing)
    {
        [videoCamera stop];
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

- (void)dealloc
{
    videoCamera.delegate = nil;
}

/////// Helper Functions ////////////////////////////////////////////////

// get four points of the pickers into opencv////////////////////////////
-(std::vector<cv::Point2f>) getFourPoints
{
    std::vector<cv::Point2f> FourPoints;
    float x1  = Picker1.center.x * cam_width    / view_width;
    float y1  = Picker1.center.y * cam_height   / view_height - offset;
    cv::Point2f Point1(x1,y1);
    FourPoints.push_back(Point1);
    
    float x2  = Picker2.center.x  * cam_width   / view_width;
    float y2  = Picker2.center.y  * cam_height  / view_height - offset;
    cv::Point2f Point2(x2,y2);
    FourPoints.push_back(Point2);
    
    float x3  = Picker3.center.x  * cam_width   / view_width;
    float y3  = Picker3.center.y  * cam_height  / view_height - offset;
    cv::Point2f Point3(x3,y3);
    FourPoints.push_back(Point3);
    
    float x4  = Picker4.center.x  * cam_width   / view_width;
    float y4  = Picker4.center.y  * cam_height  / view_height - offset;
    cv::Point2f Point4(x4,y4);
    FourPoints.push_back(Point4);
    
    return FourPoints;
}

//////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToTrackedBlobs(std::vector<Blob> &trackedBlobs,
                                          std::vector<Blob> &currentFrameBlobs,
                                          std::unordered_map<int, BOOL> &trackedIDs)
{
    trackedIDs.clear();
    for (auto &trackedBlob : trackedBlobs)
    {
        trackedBlob.blnCurrentMatchFoundOrNewBlob = false;
        trackedBlob.predictNextPosition();
        trackedIDs[trackedBlob.ID] = false;
    }
    // if there are more blobs than the blobs being tracked
    if (currentFrameBlobs.size() >= trackedBlobs.size())
    {
        // keep track of which current blob is already matched with tracked blobs
        std::unordered_map<int, BOOL> matchedCurrentBlobs;
        for (int i=0; i < currentFrameBlobs.size(); i++)
        {
            matchedCurrentBlobs[i] = false;
        }
        for (auto &trackedBlob : trackedBlobs)
        {
            // find closest blob in the current blobs
            int intIndexOfLeastDistance = 0;
            double dblLeastDistance = 100000.0;
            for (unsigned int i = 0; i < currentFrameBlobs.size(); i++)
            {
                // skip the current blob which is already matched to the tracked blob
                if (matchedCurrentBlobs[i] == true)
                {
                    continue;
                }
                double dblDistance = distanceBetweenPoints(currentFrameBlobs[i].centerPositions.back(), trackedBlob.predictedNextPosition);
                if (dblDistance < dblLeastDistance)
                {
                    dblLeastDistance = dblDistance;
                    // find which blob in the tracked blobs is the closest to current blob
                    intIndexOfLeastDistance = i;
                }
            }
            // update tracked ball information to the current frame blob
            UpdateTrackedBlob(trackedBlob, currentFrameBlobs, intIndexOfLeastDistance);
            // mark the tracked ball's ID to be true. Untracked blob's ID is false
            trackedIDs[trackedBlob.ID] = true;
            // mark the matched current blob true
            matchedCurrentBlobs[intIndexOfLeastDistance] = true;
        }
    }
    // if there are less blobs in the current frame than the blobs being tracked
    else
    {
        for (auto &currentFrameBlob : currentFrameBlobs)
        {
            int IDOfLeastDistance = 0;
            double dblLeastDistance = 100000.0;
            for (unsigned int i = 0; i < trackedBlobs.size(); i++)
            {
                // skip the current blob which is already matched to the tracked blob
                if (trackedIDs[currentFrameBlob.ID])
                {
                    continue;
                }
                double dblDistance = distanceBetweenPoints(currentFrameBlob.centerPositions.back(), trackedBlobs[i].predictedNextPosition);
                if (dblDistance < dblLeastDistance)
                {
                    dblLeastDistance    = dblDistance;
                    IDOfLeastDistance   = trackedBlobs[i].ID;
                }
            }
            // update the closest tracked blob to the current blob
            addBlobToExistingBlobs(currentFrameBlob, trackedBlobs, IDOfLeastDistance);
            // mark the tracked ball's ID to be true. Untracked blob's ID is false
            trackedIDs[IDOfLeastDistance] = true;
        }
    }
    for (auto &trackedBlob : trackedBlobs)
    {
        if (trackedIDs[trackedBlob.ID] == false)
        {
            trackedBlob.blnStillBeingTracked = false;
        }
    }
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs,
                                           std::vector<Blob> &currentFrameBlobs)
{
    for (auto &existingBlob : existingBlobs)
    {
        existingBlob.blnCurrentMatchFoundOrNewBlob = false;
        existingBlob.predictNextPosition();
    }
    for (auto &currentFrameBlob : currentFrameBlobs)
    {
        int intIndexOfLeastDistance = 0;
        double dblLeastDistance = 100000.0;
        for (unsigned int i = 0; i < existingBlobs.size(); i++)
        {
            if (existingBlobs[i].blnStillBeingTracked == true)
            {
                double dblDistance = distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);
                if (dblDistance < dblLeastDistance)
                {
                    dblLeastDistance = dblDistance;
                    intIndexOfLeastDistance = i;
                }
            }
        }
        if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 1.15)
        {
            addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
        }
        else
        {
            addNewBlob(currentFrameBlob, existingBlobs);
        }
    }
    
    for (auto &existingBlob : existingBlobs)
    {
        if (existingBlob.blnCurrentMatchFoundOrNewBlob == false)
        {
            existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
        }
        if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5)
        {
            existingBlob.blnStillBeingTracked = false;
        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void extractBlobPositionsToVector(std::vector<Blob> blobs, std::vector<cv::Point2f> &circle_coords)
{
    for (auto blob : blobs)
    {
        cv::Point2f blobPosition = blob.centerPositions.back();
        circle_coords.push_back(blobPosition);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex)
{
    cv::Point updatedPosition = currentFrameBlob.centerPositions.back();
    if(trackingStarted)
    {
        auto existingBlob = existingBlobs[intIndex];
        cv::Point mostRecentTrajPoint = existingBlob.trajectoryPoints.back();
        auto distance = cv::norm(mostRecentTrajPoint - updatedPosition);
        if(distance > minDistThresh && distance < maxDistThresh)
        {
           existingBlobs[intIndex].trajectoryPoints.push_back(updatedPosition);
        }
    }
    existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
    existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;
    existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());
    existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
    existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;
    existingBlobs[intIndex].blnStillBeingTracked = true;
    existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateTrackedBlob(Blob &trackedBlob, std::vector<Blob> &currentFrameBlobs, int &intIndex)
{
    cv::Point2f updatedPosition = cv::Point2f(currentFrameBlobs[intIndex].centerPositions.back().x,
                                              currentFrameBlobs[intIndex].centerPositions.back().y);
    auto distance = cv::norm(trackedBlob.trajectoryPoints.back() - updatedPosition);
    if(trackingStarted && distance > minDistThresh && distance < maxDistThresh)
    {
        trackedBlob.trajectoryPoints.push_back(updatedPosition);
    }
    trackedBlob.currentContour          = currentFrameBlobs[intIndex].currentContour;
    trackedBlob.currentBoundingRect     = currentFrameBlobs[intIndex].currentBoundingRect;
    trackedBlob.dblCurrentDiagonalSize  = currentFrameBlobs[intIndex].dblCurrentDiagonalSize;
    trackedBlob.dblCurrentAspectRatio   = currentFrameBlobs[intIndex].dblCurrentAspectRatio;
    trackedBlob.blnStillBeingTracked            = true;
    trackedBlob.blnCurrentMatchFoundOrNewBlob   = true;
    trackedBlob.centerPositions.push_back(currentFrameBlobs[intIndex].centerPositions.back());
}


///////////////////////////////////////////////////////////////////////////////////////////////////
void addMissingBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex)
{
    existingBlobs[intIndex].currentContour          = currentFrameBlob.currentContour;
    existingBlobs[intIndex].currentBoundingRect     = currentFrameBlob.currentBoundingRect;
    existingBlobs[intIndex].dblCurrentDiagonalSize  = currentFrameBlob.dblCurrentDiagonalSize;
    existingBlobs[intIndex].dblCurrentAspectRatio   = currentFrameBlob.dblCurrentAspectRatio;
    existingBlobs[intIndex].blnStillBeingTracked          = true;
    existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
    existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs)
{
    currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;
    existingBlobs.push_back(currentFrameBlob);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double distanceBetweenPoints(cv::Point point1, cv::Point point2)
{
    int intX = abs(point1.x - point2.x);
    int intY = abs(point1.y - point2.y);
    return(sqrt(pow(intX, 2) + pow(intY, 2)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy)
{
    for (unsigned int i = 0; i < blobs.size(); i++)
    {
        if (blobs[i].blnStillBeingTracked == true)
        {
            cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, COLORS[i%5], 2);
            int     intFontFace         =   CV_FONT_HERSHEY_SIMPLEX;
            double  dblFontScale        =   blobs[i].dblCurrentDiagonalSize / 60.0;
            int     intFontThickness    =   (int)std::round(dblFontScale * 1.0);
            cv::Point2f center(cv::Point2f(blobs[i].centerPositions.back().x,
                                           blobs[i].centerPositions.back().y));
            cv::circle(imgFrame2Copy, center, 3, RED,3, -1, 0);
//            cv::putText(imgFrame2Copy,
//                        std::to_string(i),
//                        blobs[i].centerPositions.back(),
//                        intFontFace,
//                        dblFontScale,
//                        COLORS[i%5],
//                        intFontThickness);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawTrajectoryOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy)
{
    for (unsigned int i = 0; i < blobs.size(); i++)
    {
        auto trajectory = blobs[i].trajectoryPoints;
        auto blobID = blobs[i].ID;
        for (int i=0; i<trajectory.size()-1; i++)
        {
            auto pt1 = trajectory[i];
            auto pt2 = trajectory[i+1];
            cv::line(imgFrame2Copy, pt1, pt2, COLORS[blobID % 5], 3, CV_AA);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawTrajectoriesOnImage(std::vector<std::vector<cv::Point2f>> trajectories, cv::Mat &imgFrame2Copy)
{
    int colorNum = 0;
    for (auto traj : trajectories)
    {
        for (int i=0; i<traj.size()-1; i++)
        {
            auto pt1 = traj[i];
            auto pt2 = traj[i+1];
            if (i==0)
            {
                drawAndAnnotateCircle(imgFrame2Copy, pt1, "Begin", virtualBallRadius, COLORS[colorNum % 5]);
            }
            if (i==traj.size()-2)
            {
                drawAndAnnotateCircle(imgFrame2Copy, pt2, "End", virtualBallRadius, COLORS[colorNum % 5]);
            }
            cv::line(imgFrame2Copy, pt1, pt2, COLORS[colorNum % 5], 3, CV_AA);
        }
        colorNum++;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawAndAnnotateCircle(cv::Mat &img, cv::Point2f pt, std::string text, int radius, cv::Scalar color)
{
    cv::circle(img, pt, virtualBallRadius, color, 5, CV_AA, 0);
    cv::putText(img,
                text,
                pt + cv::Point2f(10,5),
                CV_FONT_HERSHEY_PLAIN,
                1.2,
                WHITE);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void clearTrackedRecords()
{
    blobs.clear();
    trackedBlobs.clear();
    trackedIDs.clear();
    trackedBlobs.clear();
    transformed_trajectories.clear();
    circle_coords.clear();
    transformed_circles.clear();
    virtualCuePositions.clear();
    cueBallSelected         =   false;
    targetBallSelected      =   false;
    goalPositionSelected    =   false;
    trackingFirstTime       =   true;
    predictFirstTime        =   true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// convert cv::Point2f to cv::Point
std::vector<cv::Point> floatPoint2Int(std::vector<cv::Point2f> inputVec)
{
    std::vector<cv::Point> vectorToReturn;
    for (size_t i=0 ; i<inputVec.size(); i++)
    {
        vectorToReturn.push_back( cv::Point( (int)inputVec[i].x, (int)inputVec[i].y  ) );
    }
    return vectorToReturn;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Find unit vector that connects pt1 and pt2 * radius.
// This is amount of shift for the target ball
cv::Point2f FindShiftAmount(cv::Point2f pt1, cv::Point2f pt2, int radius)
{
    cv::Point2f diff = pt2 - pt1;
    float norm = (float)cv::norm(diff);
    cv::Point2f shift = cv::Point2f(diff.x * radius * 2 / norm, diff.y * radius *2 / norm);
    return shift;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Put coordinate text on an image
void PutCoordinateText(cv::Mat img, cv::Point2f point)
{
    std::string coordX      = std::to_string((int)point.x);
    std::string coordY      = std::to_string((int)point.y);
    std::string coordinate  = "(" + coordX + ", " + coordY + ")";
    cv::putText(img,
                coordinate ,
                point + cv::Point2f(5,50),
                CV_FONT_HERSHEY_PLAIN,
                1.2,
                cv::Scalar(220,20, 60));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void warpBlobTrajectories(std::vector<Blob> blobs,
               std::vector<std::vector<cv::Point2f>> &transformed_trajectories,
               cv::Mat homography)
{
    for (auto blob : blobs)
    {
        auto currentTraj = blob.trajectoryPoints;
        std::vector<cv::Point2f> transformedTraj;
        cv::perspectiveTransform( currentTraj, transformedTraj, homography);
        transformed_trajectories.push_back(transformedTraj);
    }
    
}

@end
