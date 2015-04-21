//
//  MTConnectClient.m
//  Robot
//
//  Created by William Sobel on 4/18/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import "MTConnectClient.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>


@implementation MTConnectClient

- (id) writeString: (NSString*) string {
  return [self writeChars: [string cStringUsingEncoding: NSASCIIStringEncoding]];
}

- (id) writeChars: (const char*) string {
  [_outputBuffer appendBytes: string length: strlen(string)];
  if ([_output hasSpaceAvailable]) {
    [self stream: _output handleEvent: NSStreamEventHasSpaceAvailable];
  }
  
  return self;
}

- (void) received: (NSString*) line {
  if ([line isEqualToString: @"* PING"]) {
    [self writeChars: "* PONG 10000\n"];
  }
}

- (void)stream:(NSStream *) stream handleEvent:(NSStreamEvent) event {
  switch (event) {
    case NSStreamEventHasBytesAvailable:
    {
      uint8_t buf[1024];
      unsigned long len = 0;
      len = [(NSInputStream *)stream read:buf maxLength:1024];
      if (len > 0) {
        [_inputBuffer appendString: [[NSString alloc] initWithBytes: buf
                                                      length: len
                                                      encoding: NSASCIIStringEncoding]];
        NSMutableArray *lines = [NSMutableArray arrayWithArray:
                                 [_inputBuffer componentsSeparatedByCharactersInSet:
                                  [NSCharacterSet newlineCharacterSet]]];
        // If the last object is not an empty string, then replace the input buffer.
        if ([[lines lastObject] length] > 0) {
          [_inputBuffer setString: [lines lastObject]];
          [lines removeLastObject];
        }
        
        [lines enumerateObjectsUsingBlock: ^(id obj, NSUInteger idx, BOOL *stop) {
          if ([obj length] > 0) [self received: obj];
        }];
        
      }
      break;
    }
      
    case NSStreamEventHasSpaceAvailable:
      if ([_outputBuffer length] > 0) {
        uint8_t *readBytes = (uint8_t *) [_outputBuffer mutableBytes];
        readBytes += _outputOffset; // instance variable to move pointer
        
        unsigned long len = [_outputBuffer length] - _outputOffset;
        uint8_t buf[len];
        memcpy(buf, readBytes, len);
        unsigned long written = [_output write: buf maxLength: len];
        if (written == len) {
          _outputBuffer = [[NSMutableData alloc] initWithCapacity: 4 * 1024];
          _outputOffset = 0;
        } else {
          _outputOffset += written;
        }
      }
      break;
      
    case NSStreamEventEndEncountered:
      [stream removeFromRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
      if (stream == _input)
        _input = NULL;
      else if (stream == _output)
        _output = NULL;
      [self close];
      break;
      
    case NSStreamEventErrorOccurred:
    {
      NSError *error = [stream streamError];
      NSLog(@"An error occurred: %@", error);
      [self close];
      break;
    }
      
    case NSStreamEventOpenCompleted:
      if (stream == _input) {
        _inputBuffer = [[NSMutableString alloc] init];
      }
      break;

      
    case NSStreamEventNone:
      break;
  }
  
}


- (id) initWithSocket: (CFSocketNativeHandle) handle {
  self = [super init];
  if (self) {
    _outputBuffer = [[NSMutableData alloc] initWithCapacity: 4 * 1024];
    _outputOffset = 0;
    _open = false;
    
    int flag = 1;
    setsockopt(handle, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    CFReadStreamRef readStream = NULL;
    CFWriteStreamRef writeStream = NULL;
    CFStreamCreatePairWithSocket(kCFAllocatorDefault, handle, &readStream, &writeStream);
    
    if (readStream && writeStream) {
      CFReadStreamSetProperty(readStream, kCFStreamPropertyShouldCloseNativeSocket, kCFBooleanTrue);
      CFWriteStreamSetProperty(writeStream, kCFStreamPropertyShouldCloseNativeSocket, kCFBooleanTrue);
      
      _input = (__bridge NSInputStream*) readStream;
      [_input scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode: NSDefaultRunLoopMode];
      [_input setDelegate: self];
      [_input open];
      _output = (__bridge NSOutputStream*) writeStream;
      [_output scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode: NSDefaultRunLoopMode];
      [_output setDelegate: self];
      [_output open];
      
      _open = true;
    }
  }
  
  return self;
}

- (void) close {
  if (_open) {
    if (_input != NULL) {
      [_input close];
    }
    
    if (_output != NULL) {
      [_output close];
    }
    _open = false;
    if ([_delegate respondsToSelector: @selector(clientClosed:)])
      [_delegate clientClosed: self];
  }
}

- (void) finalize {
  [self close];
  [super finalize];
}

@end
