//
//  MTConnectAdapter.h
//  Robot
//
//  Created by William Sobel on 4/18/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "MTConnectClient.h"
#import "MTConnectDataItem.h"

@interface MTConnectAdapter : NSObject <MTConnectClientDelegate>
{
  CFSocketRef _socket;
  NSMutableArray *_clients;
  NSMutableArray *_dataItems;
  
  NSDateFormatter *_utc8601Formatter;

}

@property int port;

- (id) initWithPort: (int) port;
- (id) start;
- (id) stop;
- (void) writeChangedDataItems;

- (void) addDataItem: (MTConnectDataItem*) item;

@end
