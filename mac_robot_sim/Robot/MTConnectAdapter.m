//
//  MTConnectAdapter.m
//  Robot
//
//  Created by William Sobel on 4/18/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import "MTConnectAdapter.h"
#import "MTConnectClient.h"
#import <CoreFoundation/CoreFoundation.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>


@implementation MTConnectAdapter


- (id) initWithPort: (int) port {
  self = [super init];
  
  if (self) {
    _port = port;
    _clients = [[NSMutableArray alloc] init];
    _dataItems = [[NSMutableArray alloc] init];
    
    _utc8601Formatter = [[NSDateFormatter alloc] init];
    NSCalendar *calendar = [[NSCalendar alloc] initWithCalendarIdentifier: NSCalendarIdentifierGregorian];
    [_utc8601Formatter setCalendar:calendar];
    NSLocale *locale = [[NSLocale alloc] initWithLocaleIdentifier:@"en_US_POSIX"];
    [_utc8601Formatter setLocale:locale];
    NSTimeZone *timeZone = [NSTimeZone timeZoneForSecondsFromGMT:0];
    [_utc8601Formatter setTimeZone:timeZone];
    [_utc8601Formatter setDateFormat:@"yyyy'-'MM'-'dd'T'HH':'mm':'ss'Z'"];
  }
  
  return self;
}

- (NSString*) timestamp {
  return [_utc8601Formatter stringFromDate:[NSDate date]];
}

- (NSArray*) dataItemsToLines: (NSArray*) items {
  NSMutableArray *lines = [[NSMutableArray alloc] init];
  NSMutableString *single = [[NSMutableString alloc] init];
  
  for (MTConnectDataItem* item in items) {
    if (item.separateLine)
      [lines addObject: [item stringValue]];
    else
      [single appendString: [item stringValue]];
  }
  
  if (single.length > 0)
    [lines addObject: single];
  
  return lines;
}

- (void) writeLines: (NSArray*) lines toClient: (MTConnectClient*) client at: (NSString*) time {
  for (NSString *line in lines) {
    if (line.length > 0) {
      [client writeString: [NSString stringWithFormat: @"%@%@\n", time, line]];
    }
  }
}

- (void) writeInitialData: (MTConnectClient*) client {
  [self writeLines: [self dataItemsToLines: _dataItems] toClient: client at: [self timestamp]];
}

- (void)clearChanged {
  for (MTConnectDataItem *item in _dataItems) item.changed = NO;
}

- (void) writeChangedDataItems {
  NSString *time = [self timestamp];
  NSPredicate *changed = [NSPredicate predicateWithFormat: @"changed == YES"];
  NSArray *items = [_dataItems filteredArrayUsingPredicate: changed];
  NSArray *lines = [self dataItemsToLines: items];
  if (lines.count > 0) {
    NSLog(@"Sending: %@", lines);
    for (id client in _clients) {
      [self writeLines: lines toClient: client at: time];
    }
  }
  [self clearChanged];
}

- (id) newConnectionWithSocket: (CFSocketNativeHandle) socket {
  MTConnectClient *client = [[MTConnectClient alloc] initWithSocket: socket];
  [self writeInitialData: client];
  [_clients addObject: client];
  
  return self;
}

static void AcceptCallback(CFSocketRef s, CFSocketCallBackType type, CFDataRef address, const void *data, void *info )
{
  MTConnectAdapter *adapter = (__bridge MTConnectAdapter*) info;
  if (type == kCFSocketAcceptCallBack) {
    CFSocketNativeHandle handle = *(CFSocketNativeHandle*) data;
    [adapter newConnectionWithSocket: handle];
  } else {
    NSLog(@"AcceptCallback called with type other than accept");
  }
}


- (id) start {
  if (_socket == NULL)
  {
    CFSocketContext context = {0, (__bridge void *)(self), NULL, NULL, NULL};;
    _socket = CFSocketCreate(kCFAllocatorDefault, PF_INET, SOCK_STREAM, IPPROTO_TCP, kCFSocketAcceptCallBack, AcceptCallback, &context);
    if (_socket != NULL)
    {
      int trueval = 1;
      setsockopt(CFSocketGetNative(_socket), SOL_SOCKET, SO_REUSEADDR, (void*) &trueval, sizeof(trueval));
      
      // Bind
      struct sockaddr_in addr;
      memset(&addr, 0, sizeof(addr));
      addr.sin_len = sizeof(addr);
      addr.sin_family = AF_INET;
      addr.sin_port = htons(_port);
      addr.sin_addr.s_addr = htonl(INADDR_ANY);
      NSData *address = [NSData dataWithBytes:&addr length:sizeof(addr)];
      
      if (CFSocketSetAddress(_socket, (CFDataRef) address) == kCFSocketSuccess) {
        // Add to the run loop.
        CFRunLoopRef cfrl = CFRunLoopGetCurrent();
        CFRunLoopSourceRef source = CFSocketCreateRunLoopSource(kCFAllocatorDefault, _socket, 0);
        CFRunLoopAddSource(cfrl, source, kCFRunLoopCommonModes);
        CFRelease(source);
        
        // All set to go...
        [self clearChanged];
      } else {
        perror("MTConnectAdapter start bind");
        NSLog(@"Cannot bind address");
        CFRelease(_socket);
        _socket = NULL;
      }
    }
    else
    {
      perror("MTConnect Adapter Start");
      NSLog(@"Error, cannot create socket");
    }
  } else {
    NSLog(@"Socket has already been created");
  }
  
  return self;
}

- (id) stop {
  CFSocketInvalidate(_socket);
  CFRelease(_socket);
  _socket = NULL;
  return self;
}

- (void) clientClosed: (id) client {
  [_clients removeObject: client];
}

- (void) addDataItem: (MTConnectDataItem*) item {
  [_dataItems addObject: item];
}



@end
