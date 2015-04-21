//
//  MTConnectClient.h
//  Robot
//
//  Created by William Sobel on 4/18/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import <Foundation/Foundation.h>

@protocol MTConnectClientDelegate <NSObject>

- (void) clientClosed: (id) client;

@end

@interface MTConnectClient : NSObject <NSStreamDelegate> {
  NSInputStream *_input;
  NSOutputStream *_output;
  NSMutableString *_inputBuffer;
  NSMutableData *_outputBuffer;
  
  unsigned long _outputOffset;
}

@property (weak) id delegate;
@property (readonly) bool open;

- (id) initWithSocket: (CFSocketNativeHandle) socket;
- (id) writeString: (NSString*) string;
- (id) writeChars: (const char*) string;

- (void) close;

@end
