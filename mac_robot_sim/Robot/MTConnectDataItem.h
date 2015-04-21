//
//  MTConnectDataItem.h
//  Robot
//
//  Created by William Sobel on 4/19/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface MTConnectDataItem : NSObject
{
}

@property (copy) NSString * name;
@property bool changed;
@property bool separateLine;
@property (readonly) id value;

- (id) init;
- (id) initWithName: (NSString*) name;
- setValue: (id) value;
- (NSString*) stringValue;

@end
