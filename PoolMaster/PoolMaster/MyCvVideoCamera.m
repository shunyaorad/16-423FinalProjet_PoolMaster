//
//  MyCvVideoCamera.m
//  SimpleVideo
//
//  Created by SHUN YAO on 4/20/17.
//  Copyright © 2017 CMU. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "MyCvVideoCamera.h"

@implementation MyCvVideoCamera
@synthesize customPreviewLayer = _customPreviewLayer;

// Disable orientation change
- (void)updateOrientation;
{
    // nop
}

- (void)layoutPreviewLayer;
{
    if (self.parentView != nil) {
        CALayer* layer = self.customPreviewLayer;
        CGRect bounds = self.customPreviewLayer.bounds;
        layer.position = CGPointMake(self.parentView.frame.size.width/2., self.parentView.frame.size.height/2.);
        layer.bounds = bounds;
    }
}

@end
