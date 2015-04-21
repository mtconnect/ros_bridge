//
//  AppDelegate.h
//  Robot
//
//  Created by William Sobel on 4/16/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import <Cocoa/Cocoa.h>

#import "Robot.h"

@interface AppDelegate : NSObject <NSApplicationDelegate, NSURLSessionDataDelegate, NSURLSessionTaskDelegate, RobotDelegate>
{
  NSURLSession *_session;
  NSURLSessionDataTask *_streamingTask;
  Robot *_robot;
  uint64_t _nextSequence;
}

@end

