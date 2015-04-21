//
//  MTConnectDataItem.m
//  Robot
//
//  Created by William Sobel on 4/19/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import "MTConnectDataItem.h"

@implementation MTConnectDataItem

- (id) init {
  self = [super init];
  if (self != NULL) {
    _separateLine = _changed = NO;
  }
  return self;
}

- (id) initWithName: (NSString*) name {
  self = [self init];
  if (self != NULL) {
    _name = name;
  }
  return self;
}


- setValue: (id) value {
  if (![_value isEqual: value]) {
    _changed = YES;
    _value = value;
  }
  
  return self;
}

- (NSString*) stringValue {
  if (_name != NULL && _value != NULL) {
    return [NSString stringWithFormat: @"|%@|%@", _name, _value];
  } else {
    return @"";
  }
}

@end
