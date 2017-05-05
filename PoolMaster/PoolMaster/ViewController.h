//
//  ViewController.h
//  Background Subtraction and Tracking
//
//  Created by SHUN YAO on 4/10/17.
//  Copyright © 2017 CMU. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <opencv2/highgui/ios.h>
#import "MyCvVideoCamera.h"

@interface ViewController : UIViewController<CvVideoCameraDelegate>
{
    //CvVideoCamera *videoCamera; // OpenCV class for accessing the camera
    MyCvVideoCamera *videoCamera; // overwritten with myCvCamera to disable rotation
    BOOL isCapturing;
}

// Declare internal property of videoCamera
@property (nonatomic, strong) CvVideoCamera *videoCamera;

@property (nonatomic, strong) IBOutlet UIImageView* myImageView;

@end
