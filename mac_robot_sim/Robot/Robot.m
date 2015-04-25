//
//  Robot.m
//  Robot
//
//  Created by William Sobel on 4/19/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import "Robot.h"
@import Foundation;

@implementation Robot

- initWithPort: (int) port andDelay: (int) delay {
  self = [super init];
  if (self != NULL) {
    _port = port;
    _failNext = false;
    _delay = delay;
  }
  return self;
}

- (void) notifyDelegate {
  if ([_delegate respondsToSelector: @selector(robotChanged:)])
    [_delegate robotChanged: self];
}

- (void) start {
  _adapter = [[MTConnectAdapter alloc] initWithPort: _port];
  
  [_adapter addDataItem: _avail = [[MTConnectDataItem alloc] initWithName: @"avail"]];
  [_adapter addDataItem: _mode = [[MTConnectDataItem alloc] initWithName: @"mode"]];
  [_adapter addDataItem: _exec = [[MTConnectDataItem alloc] initWithName: @"exec"]];
  [_adapter addDataItem: _system = [[MTConnectDataItem alloc] initWithName: @"system"]];
  [_adapter addDataItem: _material = [[MTConnectDataItem alloc] initWithName: @"material"]];
  [_adapter addDataItem: _materialLoad = [[MTConnectDataItem alloc] initWithName: @"material_load"]];
  [_adapter addDataItem: _materialUnload = [[MTConnectDataItem alloc] initWithName: @"material_unload"]];
  [_adapter addDataItem: _openChuck = [[MTConnectDataItem alloc] initWithName: @"open_chuck"]];
  [_adapter addDataItem: _closeChuck = [[MTConnectDataItem alloc] initWithName: @"close_chuck"]];
  [_adapter addDataItem: _openDoor = [[MTConnectDataItem alloc] initWithName: @"open_door"]];
  [_adapter addDataItem: _closeDoor = [[MTConnectDataItem alloc] initWithName: @"close_door"]];
  [_adapter addDataItem: _spindleInterlock = [[MTConnectDataItem alloc] initWithName: @"s_inter"]];
  [_adapter addDataItem: _chuckUnclamp = [[MTConnectDataItem alloc] initWithName: @"c_unclamp"]];
  
  _avail.value = @"AVAILABLE";
  _system.value = @"normal||||";
  _system.separateLine = YES;
  _mode.value = @"MANUAL";
  _exec.value = @"READY";
  _spindleInterlock.value = @"INACTIVE";
  _chuckUnclamp.value = @"INACTIVE";
  
  [self notReady];
  
  [_adapter start];
  [self notifyDelegate];
}

- (void) stop {
  [_adapter stop];
}

- (void) _update {
  [_adapter writeChangedDataItems];
  [self notifyDelegate];
}

- (void) update {
  if (_delay > 0.0) {
    NSTimer* _update = [NSTimer timerWithTimeInterval: _delay target: self
                                           selector: @selector(_update)
                                           userInfo: nil repeats: NO];
    [[NSRunLoop mainRunLoop] addTimer: _update forMode: NSDefaultRunLoopMode];
  } else {
    [self _update];
  }
}

- (void) receivedDataItem: (NSString*) name withValue: (NSString*) value {
  // Snake case the names and remove underscors _ OpenDoor NOT_READY -> openDoorNotReady
  NSString *prefix = [name stringByReplacingCharactersInRange: NSMakeRange(0,1)
                           withString: [[name substringToIndex: 1] lowercaseString]];
  NSString *suffix = [[value capitalizedString] stringByReplacingOccurrencesOfString: @"_" withString: @""];
  NSString *method = [prefix stringByAppendingString: suffix];
  
  // Get the selector and call method dynamically
  SEL selector = NSSelectorFromString(method);
  
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Warc-performSelector-leaks"
  if ([self respondsToSelector: selector]) {
    if (_delay > 0.0) {
      NSTimer* disp = [NSTimer timerWithTimeInterval: _delay target: self
                                               selector: selector
                                               userInfo: nil repeats: NO];
      [[NSRunLoop mainRunLoop] addTimer: disp forMode: NSDefaultRunLoopMode];
    } else {
      [self performSelector: selector];
    }
  }
#pragma clang diagnostic pop
}

- (void) ready {
  _materialLoad.value = @"READY";
  _materialUnload.value = @"READY";
  _openChuck.value = @"READY";
  _closeChuck.value = @"READY";
  _openDoor.value = @"READY";
  _closeDoor.value = @"READY";
  [self update];
}

- (void) notReady {
  _materialLoad.value = @"NOT_READY";
  _materialUnload.value = @"NOT_READY";
  _openChuck.value = @"NOT_READY";
  _closeChuck.value = @"NOT_READY";
  _openDoor.value = @"NOT_READY";
  _closeDoor.value = @"NOT_READY";
  [self update];
}

- (void) fault {
  _system.value = @"fault|FAIL|||A fault occurred";
  [self update];
}

- (void) clearFault {
  _system.value = @"normal|||||";
  [self update];
}

- (void) loadTheMaterial {
  if (_failNext)
    _materialLoad.value = @"FAIL";
  else
    _materialLoad.value = @"ACTIVE";
  [self update];
  _failNext = false;

  [self openTheDoor];
}

- (void) materialLoadReady {
  if ([_materialLoad.value isEqualTo: @"COMPLETE"]) {
    _materialLoad.value = @"READY";
    [self update];
  }
}

- (void) materialLoadNotReady {
  [self materialLoadReady];
}

- (void) unloadTheMaterial {
  if (_failNext)
    _materialUnload.value = @"FAIL";
  else
    _materialUnload.value = @"ACTIVE";
  [self update];
  _failNext = false;
  
  [self openTheDoor];
}

- (void) materialUnloadReady {
  if ([_materialUnload.value isEqualTo: @"COMPLETE"])
  {
    _materialUnload.value = @"READY";
    [self update];
  }
}

- (void) materialUnloadNotReady {
  [self materialUnloadReady];
}


- (void) openTheDoor {
  _openDoor.value = @"ACTIVE";
  [self update];
}

- (void) openDoorComplete {
  _openDoor.value = @"READY";
  [self update];
  
  if ([_materialLoad.value isEqualTo: @"ACTIVE"] ||
      [_materialUnload.value isEqualTo: @"ACTIVE"]) {
    sleep(1);
    [self openTheChuck];
  }
}

- (void) closeTheDoor {
  _closeDoor.value = @"ACTIVE";
  [self update];
}

- (void) closeDoorComplete {
  _closeDoor.value = @"READY";
  [self update];
  
  if ([_materialLoad.value isEqualTo: @"ACTIVE"]) {
    _materialLoad.value = @"COMPLETE";
    [self update];
  }
}

- (void) openTheChuck {
  _openChuck.value = @"ACTIVE";
  [self update];
}

- (void) openChuckComplete {
  _openChuck.value = @"READY";
  [self update];
  
  if ([_materialLoad.value isEqualTo: @"ACTIVE"]) {
    sleep(1);
    [self closeTheChuck];
  } else if ([_materialUnload.value isEqualTo: @"ACTIVE"]) {
    _materialUnload.value = @"COMPLETE";
    [self update];
  }
}

- (void) closeTheChuck {
  _closeChuck.value = @"ACTIVE";
  [self update];
}

- (void) closeChuckComplete {
  _closeChuck.value = @"READY";
  [self update];
  
  if ([_materialLoad.value isEqualTo: @"ACTIVE"]) {
    sleep(1);
    [self closeTheDoor];
  }
}

#define MakeFail(interface) \
- (void) interface ## Fail { \
  if ([_ ## interface.value isEqualTo: @"ACTIVE"]) { \
    _ ## interface.value = @"FAIL"; \
    [self update]; \
  } \
} \
- (void) interface ## Ready { \
  if ([_ ## interface.value isEqualTo: @"FAIL"]) { \
    _ ## interface.value = @"READY"; \
    [self update]; \
  } \
} \
- (void) interface ## NotReady { [self interface ## Ready]; }

MakeFail(openDoor);
MakeFail(closeDoor);
MakeFail(closeChuck);
MakeFail(openChuck);

@end
