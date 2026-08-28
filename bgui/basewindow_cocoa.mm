/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Daniel Scharstein (macOS Cocoa backend)
 *
 * Copyright (c) 2024, Middlebury College
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#import <Cocoa/Cocoa.h>

#include "basewindow.h"
#include "imageadapter.h"

#include <gutil/misc.h>
#include <gutil/thread.h>
#include <gimage/image.h>

#include <pthread.h>
#include <map>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace bgui { struct BaseWindowData; }

// --- Objective-C class declarations ---

@interface BWView : NSView
{
  @public
  bgui::BaseWindowData *bwData;
}
- (instancetype)initWithFrame:(NSRect)frame data:(bgui::BaseWindowData *)data;
@end

@interface BWWindowDelegate : NSObject <NSWindowDelegate>
{
  @public
  bgui::BaseWindowData *bwData;
}
- (instancetype)initWithData:(bgui::BaseWindowData *)data;
@end

@interface BWAppDelegate : NSObject <NSApplicationDelegate>
@end

// --- BaseWindowData ---

namespace bgui
{

struct BaseWindowData
{
  BaseWindow *parent;
  NSWindow *window;
  BWView *view;
  BWWindowDelegate *wdel;

  uint8_t *buffer;        // back buffer: clearBuffer/paintBuffer write here
  uint8_t *displayBuffer; // front buffer: drawRect reads from here
  int bufW, bufH;
  int bytesPerRow;

  int w, h;           // current window client size

  std::string info, text;
  bool top, left;

  NSFont *font;
  NSDictionary *fontAttrs;

  bool running;
  bool closed;
  pthread_mutex_t mutex;

  std::map<int, dispatch_source_t> fileWatches;
  std::map<int, int> watchFds;
  int nextWatchId;
};

} // namespace bgui

// --- Static state ---

static bool appInitialized = false;
static BWAppDelegate *appDelegate = nil;

static void initApp()
{
  if (appInitialized)
  {
    return;
  }

  appInitialized = true;

  [NSApplication sharedApplication];
  [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

  appDelegate = [[BWAppDelegate alloc] init];
  [NSApp setDelegate:appDelegate];

  NSMenu *menuBar = [[NSMenu alloc] init];
  NSMenuItem *appMenuItem = [[NSMenuItem alloc] init];
  [menuBar addItem:appMenuItem];

  NSMenu *appMenu = [[NSMenu alloc] initWithTitle:@""];
  [appMenu addItemWithTitle:@"Quit" action:@selector(terminate:) keyEquivalent:@"q"];
  [appMenuItem setSubmenu:appMenu];

  [NSApp setMainMenu:menuBar];
  [NSApp finishLaunching];
}

static int getState(NSEvent *event)
{
  int state = 0;
  NSUInteger buttons = [NSEvent pressedMouseButtons];

  if (buttons & (1 << 0)) state |= bgui::BaseWindow::button1mask;
  if (buttons & (1 << 2)) state |= bgui::BaseWindow::button2mask;
  if (buttons & (1 << 1)) state |= bgui::BaseWindow::button3mask;

  NSUInteger flags = [event modifierFlags];

  if (flags & NSEventModifierFlagShift)   state |= bgui::BaseWindow::shiftmask;
  if (flags & NSEventModifierFlagControl) state |= bgui::BaseWindow::ctrlmask;

  return state;
}

// --- BWAppDelegate ---

@implementation BWAppDelegate

- (NSApplicationTerminateReply)applicationShouldTerminate:(NSApplication *)sender
{
  for (NSWindow *window in [[NSApp windows] copy])
  {
    [window performClose:nil];
  }

  return NSTerminateCancel;
}

@end

// --- BWView ---

@implementation BWView

- (instancetype)initWithFrame:(NSRect)frame data:(bgui::BaseWindowData *)data
{
  self = [super initWithFrame:frame];

  if (self)
  {
    bwData = data;
  }

  return self;
}

- (BOOL)isFlipped           { return YES; }
- (BOOL)isOpaque             { return YES; }
- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)canBecomeKeyView    { return YES; }

- (void)drawRect:(NSRect)dirtyRect
{
  bgui::BaseWindowData *d = bwData;

  if (!d || !d->buffer)
  {
    return;
  }

  pthread_mutex_lock(&d->mutex);

  CGColorSpaceRef cs = CGColorSpaceCreateDeviceRGB();
  CGDataProviderRef provider = CGDataProviderCreateWithData(
    NULL, d->displayBuffer, (size_t)d->bufH * d->bytesPerRow, NULL);

  CGImageRef fullImg = CGImageCreate(
    d->bufW, d->bufH, 8, 32, d->bytesPerRow, cs,
    kCGBitmapByteOrderDefault | kCGImageAlphaNoneSkipLast,
    provider, NULL, false, kCGRenderingIntentDefault);

  if (fullImg)
  {
    CGRect srcRect = CGRectMake(0, 0, d->w, d->h);
    CGImageRef img = CGImageCreateWithImageInRect(fullImg, srcRect);

    if (img)
    {
      CGContextRef ctx = [[NSGraphicsContext currentContext] CGContext];
      CGContextSaveGState(ctx);
      CGContextTranslateCTM(ctx, 0, d->h);
      CGContextScaleCTM(ctx, 1, -1);
      CGContextDrawImage(ctx, CGRectMake(0, 0, d->w, d->h), img);
      CGContextRestoreGState(ctx);
      CGImageRelease(img);
    }

    CGImageRelease(fullImg);
  }

  CGDataProviderRelease(provider);
  CGColorSpaceRelease(cs);

  pthread_mutex_unlock(&d->mutex);

  if (d->text.size() > 0)
  {
    NSString *text = [NSString stringWithUTF8String:d->text.c_str()];
    NSArray *lines = [text componentsSeparatedByString:@"\n"];

    CGFloat lineH = d->font.ascender - d->font.descender;
    CGFloat totalH = lines.count * lineH;
    CGFloat maxW = 0;

    for (NSString *line in lines)
    {
      CGFloat lw = [line sizeWithAttributes:d->fontAttrs].width;

      if (lw > maxW)
      {
        maxW = lw;
      }
    }

    CGFloat x = std::max(0.0, (d->w - maxW) / 2.0);
    CGFloat y = std::max(0.0, (d->h - totalH) / 2.0);

    for (NSString *line in lines)
    {
      [line drawAtPoint:NSMakePoint(x, y) withAttributes:d->fontAttrs];
      y += lineH;
    }
  }
  else if (d->info.size() > 0)
  {
    NSString *info = [NSString stringWithUTF8String:d->info.c_str()];
    NSSize sz = [info sizeWithAttributes:d->fontAttrs];

    CGFloat x = d->left ? 0 : (d->w - sz.width);
    CGFloat y = d->top ? 0 : (d->h - sz.height);

    [info drawAtPoint:NSMakePoint(x, y) withAttributes:d->fontAttrs];
  }
}

- (void)keyDown:(NSEvent *)event
{
  NSString *chars = [event characters];

  if (!chars || chars.length == 0)
  {
    return;
  }

  unichar uc = [chars characterAtIndex:0];
  char c = 0;
  bgui::BaseWindow::SpecialKey key = bgui::BaseWindow::k_none;

  switch (uc)
  {
    case 27:
      key = bgui::BaseWindow::k_esc;
      break;

    case NSLeftArrowFunctionKey:
      key = bgui::BaseWindow::k_left;
      break;

    case NSRightArrowFunctionKey:
      key = bgui::BaseWindow::k_right;
      break;

    case NSUpArrowFunctionKey:
      key = bgui::BaseWindow::k_up;
      break;

    case NSDownArrowFunctionKey:
      key = bgui::BaseWindow::k_down;
      break;

    case NSHomeFunctionKey:
      key = bgui::BaseWindow::k_home;
      break;

    case NSEndFunctionKey:
      key = bgui::BaseWindow::k_end;
      break;

    default:
      if (uc < 128)
      {
        c = (char)uc;
      }

      break;
  }

  NSPoint loc = [self.window mouseLocationOutsideOfEventStream];
  loc = [self convertPoint:loc fromView:nil];
  bwData->parent->onKey(c, key, (int)loc.x, (int)loc.y);
}

- (void)mouseDown:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMousePressed(bgui::BaseWindow::button1,
    (int)loc.x, (int)loc.y, getState(event));
}

- (void)mouseUp:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMouseReleased(bgui::BaseWindow::button1,
    (int)loc.x, (int)loc.y, getState(event));
}

- (void)rightMouseDown:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMousePressed(bgui::BaseWindow::button3,
    (int)loc.x, (int)loc.y, getState(event));
}

- (void)rightMouseUp:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMouseReleased(bgui::BaseWindow::button3,
    (int)loc.x, (int)loc.y, getState(event));
}

- (void)otherMouseDown:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMousePressed(bgui::BaseWindow::button2,
    (int)loc.x, (int)loc.y, getState(event));
}

- (void)otherMouseUp:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMouseReleased(bgui::BaseWindow::button2,
    (int)loc.x, (int)loc.y, getState(event));
}

- (void)mouseDragged:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMouseMove((int)loc.x, (int)loc.y, getState(event));
}

- (void)rightMouseDragged:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMouseMove((int)loc.x, (int)loc.y, getState(event));
}

- (void)otherMouseDragged:(NSEvent *)event
{
  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  bwData->parent->onMouseMove((int)loc.x, (int)loc.y, getState(event));
}

- (void)scrollWheel:(NSEvent *)event
{
  CGFloat dy = [event scrollingDeltaY];

  if ([event hasPreciseScrollingDeltas])
  {
    if (std::fabs(dy) < 5.0)
    {
      return;
    }
  }
  else
  {
    if (dy == 0)
    {
      return;
    }
  }

  NSPoint loc = [self convertPoint:[event locationInWindow] fromView:nil];
  int state = getState(event);

  bgui::BaseWindow::Button btn = (dy > 0)
    ? bgui::BaseWindow::button4
    : bgui::BaseWindow::button5;

  bwData->parent->onMousePressed(btn, (int)loc.x, (int)loc.y, state);
  bwData->parent->onMouseReleased(btn, (int)loc.x, (int)loc.y, state);
}

@end

// --- BWWindowDelegate ---

@implementation BWWindowDelegate

- (instancetype)initWithData:(bgui::BaseWindowData *)data
{
  self = [super init];

  if (self)
  {
    bwData = data;
  }

  return self;
}

- (void)windowDidResize:(NSNotification *)notification
{
  NSRect frame = [[bwData->window contentView] frame];
  bwData->w = (int)frame.size.width;
  bwData->h = (int)frame.size.height;
  bwData->parent->onResize(bwData->w, bwData->h);
}

- (BOOL)windowShouldClose:(NSWindow *)sender
{
  return bwData->parent->onClose() ? YES : NO;
}

- (void)windowWillClose:(NSNotification *)notification
{
  bwData->running = false;
  bwData->closed = true;
}

@end

// --- BaseWindow C++ implementation ---

namespace bgui
{

class PaintBufferFct : public gutil::ParallelFunction
{
  public:

    PaintBufferFct(BaseWindowData *_p, const ImageAdapterBase &_im, int _x, int _y) :
      p(_p), im(_im), x(_x), y(_y)
    {
      w = std::min(static_cast<long>(p->bufW), x + im.getWidth());
    }

    void run(long start, long end, long step)
    {
      gimage::ImageU8 row(w, 1, 3);

      for (long k = start; k <= end; k += step)
      {
        im.copyInto(row, -x, k - y);

        uint8_t *line = p->buffer + k * p->bytesPerRow;

        for (int i = std::max(0, x); i < w; i++)
        {
          line[i * 4 + 0] = row.get(i, 0, 0);
          line[i * 4 + 1] = row.get(i, 0, 1);
          line[i * 4 + 2] = row.get(i, 0, 2);
          line[i * 4 + 3] = 255;
        }
      }
    }

  private:

    BaseWindowData *p;
    const ImageAdapterBase &im;
    int x, y;
    int w;
};

BaseWindow::BaseWindow(const char *title, int w, int h)
{
  @autoreleasepool
  {
    initApp();

    p = new BaseWindowData();
    p->parent = this;
    p->running = true;
    p->closed = false;
    p->top = true;
    p->left = true;
    p->nextWatchId = 1;

    pthread_mutex_init(&p->mutex, NULL);

    NSRect screenFrame = [[NSScreen mainScreen] visibleFrame];
    int maxW = (int)screenFrame.size.width;
    int maxH = (int)screenFrame.size.height;

    if (w > maxW) w = maxW;
    if (h > maxH) h = maxH;

    p->w = w;
    p->h = h;

    p->bufW = maxW;
    p->bufH = maxH;
    p->bytesPerRow = p->bufW * 4;
    p->buffer = (uint8_t *)calloc(p->bufH, p->bytesPerRow);
    p->displayBuffer = (uint8_t *)calloc(p->bufH, p->bytesPerRow);

    p->font = [[NSFont monospacedSystemFontOfSize:13
                                           weight:NSFontWeightRegular] retain];

    p->fontAttrs = [@{
      NSFontAttributeName: p->font,
      NSForegroundColorAttributeName: [NSColor whiteColor],
      NSBackgroundColorAttributeName: [NSColor blackColor]
    } retain];

    NSRect contentRect = NSMakeRect(0, 0, w, h);
    NSUInteger style = NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                       NSWindowStyleMaskResizable | NSWindowStyleMaskMiniaturizable;

    p->window = [[NSWindow alloc] initWithContentRect:contentRect
                                            styleMask:style
                                              backing:NSBackingStoreBuffered
                                                defer:NO];

    [p->window setTitle:[NSString stringWithUTF8String:title]];
    [p->window setMinSize:NSMakeSize(10, 10)];
    [p->window center];

    p->view = [[BWView alloc] initWithFrame:contentRect data:p];
    [p->window setContentView:p->view];
    [p->window makeFirstResponder:p->view];

    p->wdel = [[BWWindowDelegate alloc] initWithData:p];
    [p->window setDelegate:p->wdel];
  }
}

BaseWindow::~BaseWindow()
{
  // derived classes are expected to have done this already, see stopEventLoop()

  stopEventLoop();

  @autoreleasepool
  {
    for (auto &kv : p->fileWatches)
    {
      dispatch_source_cancel(kv.second);
    }

    for (auto &kv : p->watchFds)
    {
      close(kv.second);
    }

    pthread_mutex_destroy(&p->mutex);
    free(p->buffer);
    free(p->displayBuffer);

    [p->fontAttrs release];
    [p->font release];
    [p->wdel release];
    [p->view release];
    [p->window release];

    delete p;
  }
}

void BaseWindow::setIcon(const gimage::ImageU8 &icon)
{
  @autoreleasepool
  {
    if (icon.getWidth() == 0 || icon.getHeight() == 0)
    {
      return;
    }

    int w = (int)icon.getWidth();
    int h = (int)icon.getHeight();
    int d = icon.getDepth();

    NSBitmapImageRep *rep = [[NSBitmapImageRep alloc]
      initWithBitmapDataPlanes:NULL
                    pixelsWide:w
                    pixelsHigh:h
                 bitsPerSample:8
               samplesPerPixel:4
                      hasAlpha:YES
                      isPlanar:NO
                colorSpaceName:NSDeviceRGBColorSpace
                   bytesPerRow:w * 4
                  bitsPerPixel:32];

    uint8_t *pixels = [rep bitmapData];

    for (int k = 0; k < h; k++)
    {
      for (int i = 0; i < w; i++)
      {
        int off = (k * w + i) * 4;
        pixels[off + 0] = icon.get(i, k, 0);
        pixels[off + 1] = (d > 1) ? icon.get(i, k, 1) : icon.get(i, k, 0);
        pixels[off + 2] = (d > 2) ? icon.get(i, k, 2) : icon.get(i, k, 0);
        pixels[off + 3] = (d > 3) ? icon.get(i, k, 3) : 255;
      }
    }

    NSImage *nsIcon = [[NSImage alloc] initWithSize:NSMakeSize(w, h)];
    [nsIcon addRepresentation:rep];
    [NSApp setApplicationIconImage:nsIcon];
    [nsIcon release];
    [rep release];
  }
}

void BaseWindow::setVisible(bool show)
{
  if (!p->running)
  {
    return;
  }

  @autoreleasepool
  {
    if (show)
    {
      [p->window makeKeyAndOrderFront:nil];
      [NSApp activateIgnoringOtherApps:YES];
    }
    else
    {
      [p->window orderOut:nil];
    }
  }
}

int BaseWindow::addFileWatch(const char *path)
{
  int fd = open(path, O_RDONLY | O_EVTONLY);

  if (fd < 0)
  {
    return -1;
  }

  int watchId = p->nextWatchId++;

  dispatch_source_t source = dispatch_source_create(
    DISPATCH_SOURCE_TYPE_VNODE, fd,
    DISPATCH_VNODE_WRITE | DISPATCH_VNODE_EXTEND | DISPATCH_VNODE_ATTRIB,
    dispatch_get_main_queue());

  BaseWindowData *data = p;
  int wid = watchId;

  dispatch_source_set_event_handler(source, ^{
    if (data->running)
    {
      dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 250 * NSEC_PER_MSEC),
                     dispatch_get_main_queue(), ^{
        if (data->running)
        {
          data->parent->onFileChanged(wid);
        }
      });
    }
  });

  dispatch_resume(source);

  p->fileWatches[watchId] = source;
  p->watchFds[watchId] = fd;

  return watchId;
}

void BaseWindow::removeFileWatch(int watchid)
{
  auto it = p->fileWatches.find(watchid);

  if (it != p->fileWatches.end())
  {
    dispatch_source_cancel(it->second);
    p->fileWatches.erase(it);
  }

  auto fit = p->watchFds.find(watchid);

  if (fit != p->watchFds.end())
  {
    close(fit->second);
    p->watchFds.erase(fit);
  }
}

void BaseWindow::sendClose()
{
  if (p->closed)
  {
    return;
  }

  @autoreleasepool
  {
    [p->window performClose:nil];
  }
}

void BaseWindow::waitForClose()
{
  while (!p->closed)
  {
    @autoreleasepool
    {
      NSEvent *event = [NSApp nextEventMatchingMask:NSEventMaskAny
                                          untilDate:[NSDate distantFuture]
                                             inMode:NSDefaultRunLoopMode
                                            dequeue:YES];

      if (event)
      {
        [NSApp sendEvent:event];
      }
    }
  }
}

bool BaseWindow::isClosed()
{
  return p->closed;
}

void BaseWindow::stopEventLoop()
{
  // the event loop runs in the main thread, i.e. detaching the delegate and
  // the view is sufficient for making sure that no further callback is
  // invoked on the object that is currently being destroyed

  @autoreleasepool
  {
    if (!p->closed)
    {
      [p->window setDelegate:nil];
      [p->window close];
      p->closed=true;
    }

    p->running=false;
  }
}

void BaseWindow::getContent(gimage::ImageU8 &image)
{
  pthread_mutex_lock(&p->mutex);

  image.setSize(p->w, p->h, 3);

  for (int k = 0; k < p->h; k++)
  {
    uint8_t *line = p->displayBuffer + k * p->bytesPerRow;

    for (int i = 0; i < p->w; i++)
    {
      image.set(i, k, 0, line[i * 4 + 0]);
      image.set(i, k, 1, line[i * 4 + 1]);
      image.set(i, k, 2, line[i * 4 + 2]);
    }
  }

  pthread_mutex_unlock(&p->mutex);
}

void BaseWindow::setTitle(const char *title)
{
  @autoreleasepool
  {
    [p->window setTitle:[NSString stringWithUTF8String:title]];
  }
}

void BaseWindow::getDisplaySize(int &w, int &h)
{
  NSRect frame = [[NSScreen mainScreen] visibleFrame];
  w = (int)frame.size.width;
  h = (int)frame.size.height;
}

void BaseWindow::getSize(int &w, int &h)
{
  w = p->w;
  h = p->h;
}

void BaseWindow::setSize(int w, int h)
{
  @autoreleasepool
  {
    NSRect screenFrame = [[NSScreen mainScreen] visibleFrame];

    if (w > (int)screenFrame.size.width)  w = (int)screenFrame.size.width;
    if (h > (int)screenFrame.size.height) h = (int)screenFrame.size.height;

    [p->window setContentSize:NSMakeSize(w, h)];
    p->w = w;
    p->h = h;
  }
}

void BaseWindow::setPosition(int x, int y)
{
  @autoreleasepool
  {
    NSRect screenFrame = [[NSScreen mainScreen] frame];
    CGFloat contentH = [p->window contentRectForFrameRect:[p->window frame]].size.height;
    CGFloat cocoaY = screenFrame.size.height - y - contentH;

    if (x < 0) x = 0;
    if (cocoaY < 0) cocoaY = 0;

    [p->window setFrameOrigin:NSMakePoint(x, cocoaY)];
  }
}

int BaseWindow::getTextHeight()
{
  return (int)(p->font.ascender - p->font.descender);
}

void BaseWindow::setInfoLine(const char *text, bool top, bool left)
{
  @autoreleasepool
  {
    p->info = (text ? text : "");
    p->top = top;
    p->left = left;

    if (p->running)
    {
      [p->view setNeedsDisplay:YES];
    }
  }
}

void BaseWindow::setInfoText(const char *text)
{
  @autoreleasepool
  {
    p->text = (text ? text : "");

    if (p->running)
    {
      [p->view setNeedsDisplay:YES];
    }
  }
}

bool BaseWindow::hasInfoText()
{
  return p->text.size() > 0;
}

void BaseWindow::clearBuffer()
{
  pthread_mutex_lock(&p->mutex);
  memset(p->buffer, 0, (size_t)p->bufH * p->bytesPerRow);
  pthread_mutex_unlock(&p->mutex);
}

void BaseWindow::paintBuffer(const ImageAdapterBase &im, int x, int y)
{
  pthread_mutex_lock(&p->mutex);

  PaintBufferFct fct(p, im, x, y);
  int h = std::min(static_cast<long>(p->bufH), y + im.getHeight());
  gutil::runParallel(fct, std::max(0, y), h - 1, 1);

  pthread_mutex_unlock(&p->mutex);
}

void BaseWindow::showBuffer()
{
  @autoreleasepool
  {
    if (p->running)
    {
      pthread_mutex_lock(&p->mutex);
      std::swap(p->buffer, p->displayBuffer);
      pthread_mutex_unlock(&p->mutex);

      [p->view display];
    }
  }
}

} // namespace bgui
