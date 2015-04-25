//
//  Robot.h
//  Robot
//
//  Created by William Sobel on 4/19/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import <Foundation/Foundation.h>
#include "MTConnectAdapter.h"
@interface Robot : NSObject
{
  MTConnectAdapter *_adapter;
  int _port;
}


@property (readonly) MTConnectDataItem *avail;
@property (readonly) MTConnectDataItem *mode;
@property (readonly) MTConnectDataItem *exec;
@property (readonly) MTConnectDataItem *system;
@property (readonly) MTConnectDataItem *material;
@property (readonly) MTConnectDataItem *materialLoad;
@property (readonly) MTConnectDataItem *materialUnload;
@property (readonly) MTConnectDataItem *openChuck;
@property (readonly) MTConnectDataItem *closeChuck;
@property (readonly) MTConnectDataItem *openDoor;
@property (readonly) MTConnectDataItem *closeDoor;
@property (readonly) MTConnectDataItem *spindleInterlock;
@property (readonly) MTConnectDataItem *chuckUnclamp;
@property bool failNext;
@property double delay;

@property (weak) id delegate;

- initWithPort: (int) port andDelay: (int) delay;
- (void) start;
- (void) stop;
- (void) update;

- (void) receivedDataItem: (NSString*) name withValue: (NSString*) value;

- (void) ready;
- (void) notReady;

- (void) fault;
- (void) clearFault;

- (void) loadTheMaterial;
- (void) unloadTheMaterial;
- (void) openTheDoor;
- (void) closeTheDoor;
- (void) openTheChuck;
- (void) closeTheChuck;

@end

@protocol RobotDelegate <NSObject>

- (void) robotChanged: (Robot*) robot;

@end

