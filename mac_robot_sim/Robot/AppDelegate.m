//
//  AppDelegate.m
//  Robot
//
//  Created by William Sobel on 4/16/15.
//  Copyright (c) 2015 William Sobel. All rights reserved.
//

#import "AppDelegate.h"

@import Foundation;

@interface AppDelegate ()

@property (weak) IBOutlet NSWindow *window;
@property (weak) IBOutlet NSTextField *cncUrl;
@property (weak) IBOutlet NSTextField *adapterPort;

// Robot
@property (weak) IBOutlet NSComboBox *controllerModeBox;
@property (weak) IBOutlet NSComboBox *executionBox;
@property (weak) IBOutlet NSTextField *materialLoadField;
@property (weak) IBOutlet NSTextField *materialUnloadField;
@property (weak) IBOutlet NSTextField *openChuckField;
@property (weak) IBOutlet NSTextField *closeChuckField;
@property (weak) IBOutlet NSTextField *openDoorField;
@property (weak) IBOutlet NSTextField *closeDoorField;

// CNC
@property (weak) IBOutlet NSTextField *cncControllerModeField;
@property (weak) IBOutlet NSTextField *cncExecutionField;
@property (weak) IBOutlet NSTextField *cncSystemField;
@property (weak) IBOutlet NSTextField *cncMaterialLoadField;
@property (weak) IBOutlet NSTextField *cncMaterialUnloadField;
@property (weak) IBOutlet NSLevelIndicator *cncChuckState;
@property (weak) IBOutlet NSTextField *cncChuckStateField;
@property (weak) IBOutlet NSTextField *cncOpenChuckField;
@property (weak) IBOutlet NSTextField *cncCloseChuckField;
@property (weak) IBOutlet NSLevelIndicator *cncDoorState;
@property (weak) IBOutlet NSTextField *cncDoorStateField;
@property (weak) IBOutlet NSTextField *cncOpenDoorField;
@property (weak) IBOutlet NSTextField *cncCloseDoorField;
@property (weak) IBOutlet NSTextField *cncLinkStateField;
@property (weak) IBOutlet NSTextField *cncMaterialField;

@end

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification *)aNotification {
  // Insert code here to initialize your application
}

- (void)applicationWillTerminate:(NSNotification *)aNotification {
  // Insert code here to tear down your application
}

- (IBAction)startAdapter:(id)sender {
  // Create adapter
  _robot = [[Robot alloc] initWithPort: [_adapterPort intValue]];
  [_robot setDelegate: self];
  [_robot start];
  
  _session = [NSURLSession sessionWithConfiguration: [NSURLSessionConfiguration defaultSessionConfiguration]
                                           delegate: self delegateQueue: nil];
  
  [self startAgentStream];
}

- (void) robotChanged: (Robot*) robot {
  [_controllerModeBox setStringValue: [[_robot mode] value]];
  [_executionBox setStringValue: [[_robot exec] value]];
  [_materialLoadField setStringValue: [[_robot materialLoad] value]];
  [_materialUnloadField setStringValue: [[_robot materialUnload] value]];
  [_openChuckField setStringValue: [[_robot openChuck] value]];
  [_closeChuckField setStringValue: [[_robot closeChuck] value]];
  [_openDoorField setStringValue: [[_robot openDoor] value]];
  [_closeDoorField setStringValue: [[_robot closeDoor] value]];
}

- (IBAction) stopAdapter:(id)sender {
  [_robot stop];
  _robot = NULL;
}

- (IBAction) controllerModeChanged: (id) sender {
  [[_robot mode] setValue: [sender stringValue]];
  [_robot update];
}

- (IBAction) executionChanged: (id) sender {
  [[_robot exec] setValue: [sender stringValue]];
  [_robot update];
}

- (IBAction) readyChanged: (id) sender {
  if ([sender state] == NSOnState)
    [_robot ready];
  else
    [_robot notReady];
}

- (IBAction) faultChanged: (id) sender {
  if ([sender state] == NSOnState)
    [_robot fault];
  else
    [_robot clearFault];
}

- (IBAction) failNextChanged: (id) sender {
}

- (IBAction) loadPressed: (id) sender {
  [_robot loadTheMaterial];
}

- (IBAction) unloadPressed: (id) sender {
  [_robot unloadTheMaterial];
  
}

- (IBAction) openDoorPressed: (id) sender {
  [_robot openTheDoor];
}

- (IBAction) closeDoorPressed: (id) sender {
  [_robot closeTheDoor];
}

- (IBAction) openChuckPressed: (id) sender {
  [_robot openTheChuck];
}

- (IBAction) closeChuckPressed: (id) sender {
  [_robot closeTheChuck];
}


- (void) startAgentStream {
  // Create NSSession
  NSString *cncUrl = [[_cncUrl stringValue] stringByAppendingString: @"/current"];
  NSURL *url = [NSURL URLWithString: cncUrl];
  NSURLSessionDataTask *task = [_session dataTaskWithURL: url];
  [task resume];
}

- (void)URLSession:(NSURLSession *)session dataTask:(NSURLSessionDataTask *)dataTask
    didReceiveData:(NSData *)data {
  char buffer[data.length + 1];
  memcpy(buffer, [data bytes], [data length]);
  buffer[data.length] = '\0';
  NSString *text =[NSString stringWithUTF8String: buffer];
  
  // Sometmes it combines blocks into single docs if they are arrive close together.
  NSArray *blocks = [text componentsSeparatedByString: @"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"];
  [blocks enumerateObjectsUsingBlock:^(id xml, NSUInteger idx, BOOL *stop) {
    if ([xml length] == 0) return;
    
    NSMutableString *xmlBuffer = [[NSMutableString alloc] initWithString: xml];
    [xmlBuffer insertString:@"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n" atIndex:0];
    
    NSError *err = nil;
    NSXMLDocument *doc = [[NSXMLDocument alloc]
                          initWithXMLString: xmlBuffer
                          options: NSXMLDocumentTidyXML error: &err];
    if (doc != nil && err == nil) {
      // Get the header
      err = nil;
      NSArray *nodes = [doc nodesForXPath: @"//Header" error: &err];
      if ([nodes count] > 0) {
        NSXMLElement *header = [nodes firstObject];
        NSXMLNode *next = [header attributeForName: @"nextSequence"];
        if (next != nil) {
          _nextSequence = [[next stringValue] intValue];
        }
      } else {
        NSLog(@"Could not find header: %@", err);
      }
      
      // Get all the data items...
      err = nil;
      nodes = [doc nodesForXPath: @"//Events/*|//Condition/*" error: &err];
      [nodes enumerateObjectsUsingBlock: ^(id node, NSUInteger idx, BOOL *stop) {
        NSXMLElement *grandparent = (NSXMLElement*)[[node parent] parent];
        NSString *component = [[grandparent attributeForName: @"component"] stringValue];
        NSString *name = [node name];
        NSString *value = [node stringValue];
        NSRange range = [name rangeOfString: @"^(Unavailable|Normal|Warning|Fault)$" options: NSRegularExpressionSearch];
        if (range.location != NSNotFound) {
          value = [[NSString alloc] initWithFormat: @"%@: %@", name, value];
          name = [[[node attributeForName: @"type"] stringValue] capitalizedString];
        } else {
          NSRange range = [name rangeOfString: @"^(Open|Close)$" options: NSRegularExpressionSearch];
          if (range.location != NSNotFound) {
            NSString *suffix = [component stringByReplacingOccurrencesOfString: @"Interface" withString: @""];
            name = [name stringByAppendingString: suffix];
          }
        }
        
        NSLog(@"Data Item: %@, %@, %@ ", component, name, [node stringValue]);
        NSString *selectorName = [[NSString alloc] initWithFormat: @"cnc%@Field", name];
        SEL method = NSSelectorFromString(selectorName);
        if ([self respondsToSelector: method]) {
          NSTextField *field = [self performSelector: method];
          if (field != nil) [field setStringValue: value];
          [_robot receivedDataItem: name with: value];
        }

        if ([name hasSuffix: @"State"]) {
          int level = 0;
          if ([value isEqual: @"OPEN"])
            level = 1;
          else if ([value isEqual: @"UNLATCHED"])
            level = 2;
          else if ([value isEqual: @"CLOSED"])
            level = 3;
          
          if ([name isEqual: @"ChuckState"]) {
            [_cncChuckState setIntValue: level];
          } else if ([name isEqual: @"DoorState"]) {
            [_cncDoorState setIntValue: level];
          }
        }
      }];
    } else {
      NSLog(@"Could not parse document: %@", err);
      NSLog(@"---------------------------------------------");
      NSLog(@"Received %@", xmlBuffer);
      NSLog(@"---------------------------------------------");
      NSLog(@"---------------------------------------------");
      NSLog(@"Complete %@", text);
      NSLog(@"---------------------------------------------");
    }
  }];
}

- (void) URLSession:(NSURLSession *)session task: (NSURLSessionTask*) task didCompleteWithError:(NSError *)error {
  NSLog(@"Task completed");
  if (error) {
    NSLog(@"Error: %@", error);
    if (_streamingTask != nil) {
      [_streamingTask cancel];
      _streamingTask = nil;
    }
    NSTimer* retry = [NSTimer timerWithTimeInterval: 2.0 target: self selector: @selector(startAgentStream)
                              userInfo: nil repeats: NO];
    [[NSRunLoop mainRunLoop] addTimer: retry forMode: NSDefaultRunLoopMode];
  } else {
    // Begin streaming data...
    NSLog(@"Beginning streaming task...");
    NSString *cncUrl = [[_cncUrl stringValue] stringByAppendingString:
                        [NSString stringWithFormat: @"/sample?interval=5&count=1000&from=%lld", _nextSequence]];
    NSURL *url = [NSURL URLWithString: cncUrl];
    _streamingTask = [_session dataTaskWithURL: url];
    [_streamingTask resume];
  }
}
@end
