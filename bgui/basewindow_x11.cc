/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2016 Roboception GmbH
 * Copyright (c) 2014 Institute of Robotics and Mechatronics, German Aerospace Center
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

#include "basewindow.h"

#include <gutil/exception.h>
#include <gutil/misc.h>
#include <gutil/thread.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>

#ifdef INCLUDE_INOTIFY
#include <sys/inotify.h>
#endif

#include <pthread.h>
#include <stdint.h>
#include <string>

namespace bgui
{

struct BaseWindowData
{
  BaseWindow *parent;

  Display *display;
  int     screen;

  Window  window;
  long    event_mask;
  Atom    wm_delete_window;
  GC      gc;
  XFontStruct *font;
  XImage  *image;
  int     w, h;

#ifdef INCLUDE_INOTIFY
  int     inotify_fd;
#endif

  std::string info, text;
  bool   top, left;

  pthread_mutex_t mutex;
  pthread_t       thread;
  bool            running;
};

namespace
{

class X11Exception : public gutil::Exception
{
  public:

    X11Exception(const std::string &message) :
      gutil::Exception("X11", message) { }
};

bool initx=false;

inline BaseWindow::Button getButton(unsigned int button)
{
  switch (button)
  {
    case Button1:
      return BaseWindow::button1;

    case Button2:
      return BaseWindow::button2;

    case Button3:
      return BaseWindow::button3;

    case Button4:
      return BaseWindow::button4;

    default:
    case Button5:
      return BaseWindow::button5;
  }
}

inline int getState(unsigned int state)
{
  int ret=0;

  if (state & Button1Mask)
  {
    ret|=BaseWindow::button1mask;
  }

  if (state & Button2Mask)
  {
    ret|=BaseWindow::button2mask;
  }

  if (state & Button3Mask)
  {
    ret|=BaseWindow::button3mask;
  }

  if (state & Button4Mask)
  {
    ret|=BaseWindow::button4mask;
  }

  if (state & Button5Mask)
  {
    ret|=BaseWindow::button5mask;
  }

  if (state & ShiftMask)
  {
    ret|=BaseWindow::shiftmask;
  }

  if (state & ControlMask)
  {
    ret|=BaseWindow::ctrlmask;
  }

  return ret;
}

void drawInfoText(BaseWindowData *p)
{
  if (p->text.size() > 0)
  {
    std::vector<std::string> list;

    // get total size of text and calculate position

    gutil::split(list, p->text, '\n', false);

    int x=0, y=0;
    int w=0, h=0;

    for (size_t i=0; i<list.size(); i++)
    {
      if (list[i].size() > 0)
      {
        w=std::max(w, XTextWidth(p->font, list[i].c_str(), list[i].size()));
      }

      h+=p->font->ascent+p->font->descent;
    }

    if (p->w > w)
    {
      x+=(p->w-w)/2;
    }

    if (p->h > h)
    {
      y+=(p->h-h)/2;
    }

    XGCValues values;

    memset(&values, 0, sizeof(XGCValues));
    XGetGCValues(p->display, p->gc, GCForeground | GCBackground, &values);

    XSetForeground(p->display, p->gc, values.background);
    XFillRectangle(p->display, p->window, p->gc, x, y, w, h);
    XSetForeground(p->display, p->gc, values.foreground);

    // draw multi line text

    for (size_t i=0; i<list.size(); i++)
    {
      if (list[i].size() > 0)
      {
        XDrawImageString(p->display, p->window, p->gc, x, y+p->font->ascent+
                         i*(p->font->ascent+p->font->descent), list[i].c_str(),
                         list[i].size());
      }
    }
  }
  else if (p->info.size() > 0)
  {
    // alternatively, draw one line of text at specified position

    if (p->top)
    {
      if (p->left)
      {
        XDrawImageString(p->display, p->window, p->gc, 0, p->font->ascent,
                         p->info.c_str(), p->info.size());
      }
      else
      {
        int w=XTextWidth(p->font, p->info.c_str(), p->info.size());
        XDrawImageString(p->display, p->window, p->gc, p->w-w, p->font->ascent,
                         p->info.c_str(), p->info.size());
      }
    }
    else
    {
      if (p->left)
      {
        XDrawImageString(p->display, p->window, p->gc, 0,
                         p->h-p->font->descent+1, p->info.c_str(), p->info.size());
      }
      else
      {
        int w=XTextWidth(p->font, p->info.c_str(), p->info.size());
        XDrawImageString(p->display, p->window, p->gc, p->w-w,
                         p->h-p->font->descent+1, p->info.c_str(), p->info.size());
      }
    }
  }
}

bool handleX11Event(BaseWindowData *p, XEvent &event)
{
  bool cont=true;

  switch (event.type)
  {
    case Expose:
      pthread_mutex_lock(&(p->mutex));

      XPutImage(p->display, p->window, p->gc, p->image, event.xexpose.x,
                event.xexpose.y, event.xexpose.x, event.xexpose.y,
                event.xexpose.width, event.xexpose.height);

      if (event.xexpose.count == 0)
      {
        drawInfoText(p);
      }

      pthread_mutex_unlock(&(p->mutex));
      break;

    case ConfigureNotify:
      {
        int w=event.xconfigure.width;
        int h=event.xconfigure.height;

        // collect all but the last resize event

        while (XCheckWindowEvent(p->display, p->window, StructureNotifyMask,
                                 &event))
        {
          if (event.type == ConfigureNotify)
          {
            w=event.xconfigure.width;
            h=event.xconfigure.height;
          }
        }

        if (w != p->w || h != p->h)
        {
          p->w=w;
          p->h=h;

          p->parent->onResize(p->w, p->h);
        }
      }
      break;

    case ButtonPress:
      p->parent->onMousePressed(getButton(event.xbutton.button),
                                event.xbutton.x, event.xbutton.y, getState(event.xbutton.state));
      break;

    case ButtonRelease:
      p->parent->onMouseReleased(getButton(event.xbutton.button),
                                 event.xbutton.x, event.xbutton.y, getState(event.xbutton.state));
      break;

    case MotionNotify:

      // collect all but the last motion event

      while (XCheckWindowEvent(p->display, p->window, ButtonMotionMask, &event));

      // only use the most up to date motion event

      p->parent->onMouseMove(event.xmotion.x, event.xmotion.y,
                             getState(event.xmotion.state));
      break;

    case KeyPress:
      {
        int n;
        char buffer[8];
        KeySym key;
        BaseWindow::SpecialKey sk=BaseWindow::k_none;

        n=XLookupString(&(event.xkey), buffer, 8, &key, 0);

        switch (key)
        {
          case XK_Escape:
            sk=BaseWindow::k_esc;
            break;

          case XK_Left:
            sk=BaseWindow::k_left;
            break;

          case XK_Right:
            sk=BaseWindow::k_right;
            break;

          case XK_Up:
            sk=BaseWindow::k_up;
            break;

          case XK_Down:
            sk=BaseWindow::k_down;
            break;

          case XK_Home:
            sk=BaseWindow::k_home;
            break;

          case XK_End:
            sk=BaseWindow::k_end;
            break;
        }

        if (n == 0)
        {
          buffer[0]='\0';
        }

        if (n > 0 || sk != BaseWindow::k_none)
        {
          p->parent->onKey(buffer[0], sk, event.xkey.x, event.xkey.y);
        }
      }
      break;

    case ClientMessage:
      if (event.xclient.data.l[0] == static_cast<long>(p->wm_delete_window))
      {
        if (p->parent->onClose())
        {
          pthread_mutex_lock(&(p->mutex));

          XUnmapWindow(p->display, p->window);
          cont=false;

          pthread_mutex_unlock(&(p->mutex));
        }
      }

      break;
  }

  return cont;
}

void *eventLoop(void *arg)
{
  BaseWindowData *p=static_cast<BaseWindowData *>(arg);
  bool run=true;

#ifdef INCLUDE_INOTIFY
  int x11_fd=ConnectionNumber(p->display);
#endif

  while (run)
  {
#ifdef INCLUDE_INOTIFY

    if (XPending(p->display) == 0 && p->inotify_fd != 0)
    {
      // wait for X11 and inotify events at the same time

      fd_set fds;
      const int buflen=1024;
      char buffer[buflen];

      FD_ZERO(&fds);
      FD_SET(x11_fd, &fds);
      FD_SET(p->inotify_fd, &fds);

      select(std::max(x11_fd, p->inotify_fd)+1, &fds, 0, 0, 0);

      // handle inotify events

      int i=0, s=read(p->inotify_fd, buffer, buflen);

      while (i < s)
      {
        struct inotify_event *event=reinterpret_cast<struct inotify_event *>(buffer+i);

        usleep(250000);

        p->parent->onFileChanged(event->wd);

        i+=sizeof(struct inotify_event)+event->len;
      }

      // handle X11 events

      if (XPending(p->display) > 0)
      {
        XEvent event;

        XNextEvent(p->display, &event);
        run=handleX11Event(p, event);
      }
    }
    else
#endif
    {
      // wait for and handle X11 events only

      XEvent event;

      XNextEvent(p->display, &event);
      run=handleX11Event(p, event);
    }
  }

  p->running=false;

  XUnmapWindow(p->display, p->window);
  XFlush(p->display);

  return 0;
}

}

BaseWindow::BaseWindow(const char *title, int w, int h)
{
  // set multi-threading

  if (!initx)
  {
    initx=true;

    if (XInitThreads() == 0)
    {
      throw X11Exception("Cannot initialize XLib for multi-threading");
    }
  }

  // allocate memory

  p=new BaseWindowData();
  p->parent=this;

  // connect to X server

  p->display=XOpenDisplay(0);

  if (p->display == 0)
  {
    delete p;
    throw X11Exception("Cannot connect to X display");
  }

  // limit window size to display size

  p->screen=DefaultScreen(p->display);
  int maxw=DisplayWidth(p->display, p->screen);
  int maxh=DisplayHeight(p->display, p->screen);

  p->w=w=std::min(w, maxw);
  p->h=h=std::min(h, maxh);

  // create window

  p->window=XCreateSimpleWindow(p->display,
                                RootWindow(p->display, p->screen),
                                0, 0, w, h, 0,
                                BlackPixel(p->display, p->screen),
                                WhitePixel(p->display, p->screen));

  XSetWindowBackgroundPixmap(p->display, p->window, None);

  // set hints

  {
    XSizeHints    *size_hints;
    XWMHints      *wm_hints;
    XClassHint    *class_hints;
    XTextProperty windowname, iconname;

    memset(&windowname, 0, sizeof(XTextProperty));
    memset(&iconname, 0, sizeof(XTextProperty));

    if (XStringListToTextProperty(const_cast<char **>(&title), 1, &windowname) == 0)
    {
      throw X11Exception("Allocation for title failed");
    }

    if (XStringListToTextProperty(const_cast<char **>(&title), 1, &iconname) == 0)
    {
      throw X11Exception("Allocation for icon name failed");
    }

    size_hints=XAllocSizeHints();
    wm_hints=XAllocWMHints();
    class_hints=XAllocClassHint();

    if (size_hints == 0 || wm_hints == 0 || class_hints == 0)
    {
      throw X11Exception("Allocation failed");
    }

    size_hints->flags=PSize | PMinSize | PMaxSize;
    size_hints->min_width=10;
    size_hints->min_height=10;
    size_hints->max_width=maxw;
    size_hints->max_height=maxh;

    wm_hints->flags=StateHint | InputHint;
    wm_hints->initial_state=NormalState;
    wm_hints->input=True;

    class_hints->res_name=const_cast<char *>("bwindow");
    class_hints->res_class=const_cast<char *>("bgui");

    XSetWMProperties(p->display, p->window, &windowname, &iconname,
                     0, 0, size_hints, wm_hints, class_hints);

    XFree(size_hints);
    XFree(wm_hints);
    XFree(class_hints);
    XFree(windowname.value);
    XFree(iconname.value);
  }

  p->wm_delete_window=XInternAtom(p->display, "WM_DELETE_WINDOW", False);

  if (XSetWMProtocols(p->display, p->window, &(p->wm_delete_window), 1) == 0)
  {
    throw X11Exception("Cannot set WM protocol");
  }

  // set event mask

  p->event_mask=ExposureMask|StructureNotifyMask|ButtonPressMask|
                ButtonReleaseMask|ButtonMotionMask|KeyPressMask;

  XSelectInput(p->display, p->window, p->event_mask);

  // create graphics context

  p->gc=XCreateGC(p->display, p->window, 0, 0);

  XSetBackground(p->display, p->gc, BlackPixel(p->display, p->screen));
  XSetForeground(p->display, p->gc, WhitePixel(p->display, p->screen));

  p->font=XQueryFont(p->display, XGContextFromGC(p->gc));

  // create XImage buffer

  {
    XWindowAttributes wattr;

    XGetWindowAttributes(p->display, p->window, &wattr);

    p->image=XCreateImage(p->display, wattr.visual, wattr.depth, ZPixmap, 0,
                          0, maxw, maxh, 8, 0);

    if (gutil::isMSBFirst())
    {
      p->image->byte_order=MSBFirst;
    }
    else
    {
      p->image->byte_order=LSBFirst;
    }

    if (p->image != 0)
    {
      p->image->data=(char *) calloc(p->image->bytes_per_line*p->image->height, 1);
    }

    if (p->image == 0 || p->image->data == 0)
    {
      throw X11Exception("Allocation of image failed");
    }
  }

#ifdef INCLUDE_INOTIFY
  // initialise inotify pointer

  p->inotify_fd=0;
#endif

  // init mutex and start event loop

  if (pthread_mutex_init(&(p->mutex), 0) != 0)
  {
    throw X11Exception("Cannot initialize mutex");
  }

  if (pthread_create(&(p->thread), 0, eventLoop, p) == 0)
  {
    p->running=true;
  }
  else
  {
    throw X11Exception("Cannot create event thread");
  }
}

BaseWindow::~BaseWindow()
{
  sendClose();
  waitForClose();

  pthread_mutex_destroy(&(p->mutex));

#ifdef INCLUDE_INOTIFY

  if (p->inotify_fd != 0)
  {
    close(p->inotify_fd);
  }

#endif

  XDestroyImage(p->image);
  XFreeFontInfo(0, p->font, 1);
  XFreeGC(p->display, p->gc);
  XDestroyWindow(p->display, p->window);
  XCloseDisplay(p->display);

  delete p;
}

void BaseWindow::setVisible(bool show)
{
  pthread_mutex_lock(&(p->mutex));

  if (show)
  {
    if (p->running)
    {
      XMapWindow(p->display, p->window);
    }
  }
  else
  {
    XUnmapWindow(p->display, p->window);
  }

  XFlush(p->display);

  pthread_mutex_unlock(&(p->mutex));
}

int BaseWindow::addFileWatch(const char *path)
{
#ifdef INCLUDE_INOTIFY
  int ret=-1;

  if (p->inotify_fd == 0)
  {
    p->inotify_fd=inotify_init1(IN_NONBLOCK);
  }

  if (p->inotify_fd != 0)
  {
    ret=inotify_add_watch(p->inotify_fd, path, IN_CLOSE_WRITE);
  }

  return ret;
#else
  return 0;
#endif
}

void BaseWindow::removeFileWatch(int watchid)
{
#ifdef INCLUDE_INOTIFY

  if (p->inotify_fd != 0 && watchid >= 0)
  {
    inotify_rm_watch(p->inotify_fd, watchid);
  }

#endif
}

void BaseWindow::sendClose()
{
  XEvent event;

  if (p->running)
  {
    pthread_mutex_lock(&(p->mutex));

    // request to close the window

    if (p->display != NULL)
    {
      memset(&event, 0, sizeof(XEvent));
      event.type=ClientMessage;
      event.xclient.display=p->display;
      event.xclient.window=p->window;
      event.xclient.message_type=p->wm_delete_window;
      event.xclient.format=32;
      event.xclient.data.l[0]=p->wm_delete_window;

      XSendEvent(p->display, p->window, False, 0, &event);
      XFlush(p->display);
    }

    pthread_mutex_unlock(&(p->mutex));
  }
}

void BaseWindow::waitForClose()
{
  if (p->running)
  {
    pthread_join(p->thread, 0);
    p->running=false;
  }
}

bool BaseWindow::isClosed()
{
  return !p->running;
}

void BaseWindow::setTitle(const char *title)
{
  XTextProperty name;

  pthread_mutex_lock(&(p->mutex));

  memset(&name, 0, sizeof(XTextProperty));

  XStringListToTextProperty(const_cast<char **>(&title), 1, &name);
  XSetWMName(p->display, p->window, &name);
  XFree(name.value);

  pthread_mutex_unlock(&(p->mutex));
}

void BaseWindow::getDisplaySize(int &w, int &h)
{
  w=DisplayWidth(p->display, p->screen);
  h=DisplayHeight(p->display, p->screen);
}

void BaseWindow::getSize(int &w, int &h)
{
  pthread_mutex_lock(&(p->mutex));

  w=p->w;
  h=p->h;

  pthread_mutex_unlock(&(p->mutex));
}

void BaseWindow::setSize(int w, int h)
{
  pthread_mutex_lock(&(p->mutex));

  w=std::min(w, DisplayWidth(p->display, p->screen));
  h=std::min(h, DisplayHeight(p->display, p->screen));

  XResizeWindow(p->display, p->window, w, h);

  pthread_mutex_unlock(&(p->mutex));
}

void BaseWindow::setPosition(int x, int y)
{
  pthread_mutex_lock(&(p->mutex));

  x=std::max(x, 0);
  y=std::max(y, 0);

  x=std::min(x, DisplayWidth(p->display, p->screen)-p->w);
  y=std::min(y, DisplayHeight(p->display, p->screen)-p->h);

  XMoveWindow(p->display, p->window, x, y);

  pthread_mutex_unlock(&(p->mutex));
}

int BaseWindow::getTextHeight()
{
  return p->font->ascent+p->font->descent;
}

void BaseWindow::setInfoLine(const char *text, bool top, bool left)
{
  if (strlen(text) == 0 && p->info.size() == 0)
  {
    return;
  }

  pthread_mutex_lock(&(p->mutex));

  // determine size of old info text

  int w=0, h=0;

  if (p->info.size() > 0)
  {
    w=XTextWidth(p->font, p->info.c_str(), p->info.size());
    h=p->font->ascent+p->font->descent;
  }

  // clear old std::string completely, if position has changed

  if (p->info.size() > 0 && (top != p->top || left != p->left))
  {
    if (p->top)
    {
      if (p->left)
      {
        XPutImage(p->display, p->window, p->gc, p->image, 0, 0, 0, 0, w, h);
      }
      else
        XPutImage(p->display, p->window, p->gc, p->image, p->w-w, 0, p->w-w, 0,
                  w, h);
    }
    else
    {
      if (p->left)
        XPutImage(p->display, p->window, p->gc, p->image, 0, p->h-h, 0, p->h-h,
                  w, h);
      else
        XPutImage(p->display, p->window, p->gc, p->image, p->w-w, p->h-h,
                  p->w-w, p->h-h, w, h);
    }

    w=0;
    h=0;
  }

  // store and draw new info text

  p->info=text;
  p->top=top;
  p->left=left;

  drawInfoText(p);

  // clear overlapping parts of old info text

  if (w > 0 && h > 0)
  {
    int wn=XTextWidth(p->font, p->info.c_str(), p->info.size());

    w-=wn;

    if (w > 0)
    {
      if (p->top)
      {
        if (p->left)
          XPutImage(p->display, p->window, p->gc, p->image, wn, 0, wn, 0, w,
                    h);
        else
          XPutImage(p->display, p->window, p->gc, p->image, p->w-wn-w, 0,
                    p->w-wn-w, 0, w, h);
      }
      else
      {
        if (p->left)
          XPutImage(p->display, p->window, p->gc, p->image, wn, p->h-h, wn,
                    p->h-h, w, h);
        else
          XPutImage(p->display, p->window, p->gc, p->image, p->w-wn-w,
                    p->h-h, p->w-wn-w, p->h-h, w, h);
      }
    }
  }

  pthread_mutex_unlock(&(p->mutex));
}

void BaseWindow::setInfoText(const char *text)
{
  if (strlen(text) == 0 && p->text.size() == 0)
  {
    return;
  }

  pthread_mutex_lock(&(p->mutex));

  // clear completely

  XPutImage(p->display, p->window, p->gc, p->image, 0, 0, 0, 0, p->w, p->h);

  // store text

  p->text=text;

  // draw text

  drawInfoText(p);

  pthread_mutex_unlock(&(p->mutex));
}

bool BaseWindow::hasInfoText()
{
  return p->text.size() != 0;
}

void BaseWindow::clearBuffer()
{
  pthread_mutex_lock(&(p->mutex));

  memset(p->image->data, 0, p->image->bytes_per_line*p->image->height);

  pthread_mutex_unlock(&(p->mutex));
}

namespace
{

inline void getShiftFromMask(unsigned long mask, int &left_shift,
                             int &right_shift)
{
  int s=0;

  while ((mask & (1<<s)) == 0 && s < 32)
  {
    s++;
  }

  while ((mask & (1<<s)) != 0 && s < 32)
  {
    s++;
  }

  if (s >= 32)
  {
    s=0;
  }

  if (s >= 8)
  {
    left_shift=s-8;
    right_shift=0;
  }
  else
  {
    left_shift=0;
    right_shift=8-s;
  }
}

inline uint32_t shiftAndMaskValue(unsigned char v, int left_shift,
                                  int right_shift, unsigned long mask)
{
  return ((static_cast<uint32_t>(v) << left_shift) >> right_shift) & mask;
}

inline unsigned char inverseShiftAndMaskValue(uint32_t v, int left_shift,
    int right_shift, unsigned long mask)
{
  return static_cast<unsigned char>(((v & mask)<<right_shift)>>left_shift);
}

}

void BaseWindow::getContent(gimage::ImageU8 &image)
{
  int rls, rrs;
  int gls, grs;
  int bls, brs;

  // store image in buffer

  getShiftFromMask(p->image->red_mask, rls, rrs);
  getShiftFromMask(p->image->green_mask, gls, grs);
  getShiftFromMask(p->image->blue_mask, bls, brs);

  pthread_mutex_lock(&(p->mutex));

  image.setSize(p->w, p->h, 3);

  for (int k=0; k<p->h; k++)
  {
    char *line=p->image->data+k*p->image->bytes_per_line;

    for (int i=0; i<p->w; i++)
    {
      uint32_t v=0;

      switch (p->image->bitmap_unit)
      {
        case 8:
          v=reinterpret_cast<uint8_t *>(line)[i];
          break;

        case 16:
          v=reinterpret_cast<uint16_t *>(line)[i];
          break;

        case 32:
          v=reinterpret_cast<uint32_t *>(line)[i];
          break;
      }

      image.set(i, k, 0, inverseShiftAndMaskValue(v, rls, rrs, p->image->red_mask));
      image.set(i, k, 1, inverseShiftAndMaskValue(v, gls, grs, p->image->green_mask));
      image.set(i, k, 2, inverseShiftAndMaskValue(v, bls, brs, p->image->blue_mask));
    }
  }

  pthread_mutex_unlock(&(p->mutex));
}

class PaintBufferFct : public gutil::ParallelFunction
{
  public:

    PaintBufferFct(BaseWindowData *_p, const ImageAdapterBase &_im, int _x, int _y) :
      p(_p), im(_im), x(_x), y(_y)
    {
      // store image in buffer

      getShiftFromMask(p->image->red_mask, rls, rrs);
      getShiftFromMask(p->image->green_mask, gls, grs);
      getShiftFromMask(p->image->blue_mask, bls, brs);

      w=std::min(static_cast<long>(p->image->width), x+im.getWidth());
    }

    void run(long start, long end, long step)
    {
      gimage::ImageU8 buffer(w, 1, 3);

      for (long k=start; k<=end; k+=step)
      {
        im.copyInto(buffer, -x , k-y);

        char *line=p->image->data+k*p->image->bytes_per_line;

        for (int i=std::max(0, x); i<w; i++)
        {
          uint32_t v;

          v=shiftAndMaskValue(buffer.get(i, 0, 0), rls, rrs, p->image->red_mask);
          v|=shiftAndMaskValue(buffer.get(i, 0, 1), gls, grs, p->image->green_mask);
          v|=shiftAndMaskValue(buffer.get(i, 0, 2), bls, brs, p->image->blue_mask);

          switch (p->image->bitmap_unit)
          {
            case 8:
              reinterpret_cast<uint8_t *>(line)[i]=static_cast<uint8_t>(v);
              break;

            case 16:
              reinterpret_cast<uint16_t *>(line)[i]=static_cast<uint16_t>(v);
              break;

            case 32:
              reinterpret_cast<uint32_t *>(line)[i]=v;
              break;
          }
        }
      }
    }

  private:

    BaseWindowData *p;
    const ImageAdapterBase &im;
    int x, y;

    int rls, rrs;
    int gls, grs;
    int bls, brs;
    int w;
};

void BaseWindow::paintBuffer(const ImageAdapterBase &im, int x, int y)
{
  pthread_mutex_lock(&(p->mutex));

  PaintBufferFct fct(p, im, x, y);

  int h=std::min(static_cast<long>(p->image->height), y+im.getHeight());
  gutil::runParallel(fct, std::max(0, y), h-1, 1);

  pthread_mutex_unlock(&(p->mutex));
}

void BaseWindow::showBuffer()
{
  pthread_mutex_lock(&(p->mutex));

  if (p->running)
  {
    XPutImage(p->display, p->window, p->gc, p->image, 0, 0, 0, 0, p->w, p->h);
    drawInfoText(p);
  }

  pthread_mutex_unlock(&(p->mutex));
}

}
