//
//  Robot.m
//  Robot
//
//  Created by William Sobel on 4/19/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import "Robot.h"

@implementation Robot

- initWithPort: (int) port {
  self = [super init];
  if (self != NULL) {
    _port = port;
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
  
  [_avail setValue: @"AVAILABLE"];
  [_system setValue: @"normal||||"];
  [_system setSeparateLine: YES];
  [_mode setValue: @"MANUAL"];
  [_exec setValue: @"READY"];
  [_materialLoad setValue: @"NOT_READY"];
  [_materialUnload setValue: @"NOT_READY"];
  [_openChuck setValue: @"NOT_READY"];
  [_closeChuck setValue: @"NOT_READY"];
  [_openDoor setValue: @"NOT_READY"];
  [_closeDoor setValue: @"NOT_READY"];
  [_spindleInterlock setValue: @"INACTIVE"];
  [_chuckUnclamp setValue: @"INACTIVE"];
  
  [_adapter start];
  [self notifyDelegate];
}

- (void) stop {
  [_adapter stop];
}

- (void) update {
  [_adapter writeChangedDataItems];
  [self notifyDelegate];
}

- (void) receivedDataItem: (NSString*) name with: (NSString*) value {
  NSString *method = [[name stringByReplacingCharactersInRange: NSMakeRange(0,1)
                                 withString: [[name substringToIndex: 1] lowercaseString]]
                      stringByAppendingString: [[value capitalizedString]
                                                stringByReplacingOccurrencesOfString: @"_" withString:@""]];
  SEL selector = NSSelectorFromString(method);
  if ([self respondsToSelector: selector])
    [self performSelector: selector];
}

- (void) ready {
  [_materialLoad setValue: @"READY"];
  [_materialUnload setValue: @"READY"];
  [_openChuck setValue: @"READY"];
  [_closeChuck setValue: @"READY"];
  [_openDoor setValue: @"READY"];
  [_closeDoor setValue: @"READY"];
  [self update];
}

- (void) notReady {
  [_materialLoad setValue: @"NOT_READY"];
  [_materialUnload setValue: @"NOT_READY"];
  [_openChuck setValue: @"NOT_READY"];
  [_closeChuck setValue: @"NOT_READY"];
  [_openDoor setValue: @"NOT_READY"];
  [_closeDoor setValue: @"NOT_READY"];
  [self update];
}

- (void) fault {
  [_system setValue: @"fault|FAIL|||A fault occurred"];
  [self update];
}

- (void) clearFault {
  [_system setValue: @"normal|||||"];
  [self update];
}

- (void) loadTheMaterial {
  [_materialLoad setValue:@"ACTIVE"];
  [self update];

  [self openTheDoor];
}

- (void) materialLoadReady {
  if ([[_materialLoad value] isEqualTo: @"COMPLETE"]) {
    [_materialLoad setValue:@"READY"];
    [self update];
  }
}

- (void) materialLoadNotReady {
  [self materialLoadReady];
}


- (void) unloadTheMaterial {
  [_materialUnload setValue:@"ACTIVE"];
  [self update];
  
  [self openTheDoor];
}

- (void) materialUnloadReady {
  if ([[_materialUnload value] isEqualTo: @"COMPLETE"])
  {
    [_materialUnload setValue:@"READY"];
    [self update];
  }
}

- (void) materialUnloadNotReady {
  [self materialUnloadReady];
}


- (void) openTheDoor {
  [_openDoor setValue: @"ACTIVE"];
  [self update];
}

- (void) openDoorComplete {
  [_openDoor setValue: @"READY"];
  [self update];
  
  if ([[_materialLoad value] isEqualTo: @"ACTIVE"] ||
      [[_materialUnload value] isEqualTo: @"ACTIVE"]) {
    sleep(1);
    [self openTheChuck];
  }
}

- (void) closeTheDoor {
  [_closeDoor setValue: @"ACTIVE"];
  [self update];
}

- (void) closeDoorComplete {
  [_closeDoor setValue: @"READY"];
  [self update];
  
  if ([[_materialLoad value] isEqualTo: @"ACTIVE"]) {
    [_materialLoad setValue: @"COMPLETE"];
    [self update];
  }
}

- (void) openTheChuck {
  [_openChuck setValue: @"ACTIVE"];
  [self update];
}

- (void) openChuckComplete {
  [_openChuck setValue: @"READY"];
  [self update];
  
  
  if ([[_materialLoad value] isEqualTo: @"ACTIVE"]) {
    sleep(1);
    [self closeTheChuck];
  } else if ([[_materialUnload value] isEqualTo: @"ACTIVE"]) {
    [_materialUnload setValue: @"COMPLETE"];
    [self update];
  }
}

- (void) closeTheChuck {
  [_closeChuck setValue: @"ACTIVE"];
  [self update];
}

- (void) closeChuckComplete {
  [_closeChuck setValue: @"READY"];
  [self update];
  
  if ([[_materialLoad value] isEqualTo: @"ACTIVE"]) {
    sleep(1);
    [self closeTheDoor];
  }
}

@end
