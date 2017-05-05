//
//  MyCvVideoCamera.h
//  SimpleVideo
//
//  Created by SHUN YAO on 4/20/17.
//  Copyright © 2017 CMU. All rights reserved.
//

#ifndef MyCvVideoCamera_h
#define MyCvVideoCamera_h

#import <opencv2/highgui/cap_ios.h>

@interface MyCvVideoCamera : CvVideoCamera

- (void)updateOrientation;
- (void)layoutPreviewLayer;

@property (nonatomic, retain) CALayer *customPreviewLayer;

@end

#endif /* MyCvVideoCamera_h */
