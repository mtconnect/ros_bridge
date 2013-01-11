//
//  main.m
//  Robot
//
//  Created by William Sobel on 12/17/12.
//  Copyright (c) 2012 William Sobel. All rights reserved.
//

#import <Cocoa/Cocoa.h>

#import <MacRuby/MacRuby.h>

int main(int argc, char *argv[])
{
  return macruby_main("rb_main.rb", argc, argv);
}
