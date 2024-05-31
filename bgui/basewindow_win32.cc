/*
 * This file is part of the Computer Vision Toolkit (cvkit).
 *
 * Author: Heiko Hirschmueller
 *
 * Copyright (c) 2014, Institute of Robotics and Mechatronics, German Aerospace Center
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

#include <windows.h>
#include <windowsx.h>
#include <commctrl.h>
#include <cstdlib>
#include <string>
#include <algorithm>

#undef min
#undef max

/*
  NOTE: In contrast to the X11 implementation of this classe, the event loop of the
  WIN32 implementation is not executed in a separate thread but in the method
  waitForClose(). This means that only one window can be open at a time. However,
  this is sufficient for the tool sv.
*/

namespace bgui
{

struct BaseWindowData
{
  BaseWindow    *base;
  HINSTANCE     hinstance;
  HWND          hwnd;           // pointer to window
  int           xb, yb;         // size of borders in x and y direction
  int           th;             // height of text in pixel
  HBITMAP       bitmap;         // bitmap as background buffer
  unsigned char *bpixel;        // BGR pixel array of bitmap
  int           bw, bh;         // width and height of bitmap
  int           bytes_per_line; // size of row in bytes, including padding

  std::string   info, text;     // optional info line and multi line text
  bool          top, left;      // location of info line
};

namespace
{

struct BaseWindowData *bp;

class WindowsException : public gutil::Exception
{
  public:

    WindowsException(const std::string &message):
      gutil::Exception("Windows", message) { }
};

void drawInfoText(HWND hwnd, HDC hdc, BaseWindowData *p)
{
  HFONT fixed_font=static_cast<HFONT>(GetStockObject(ANSI_FIXED_FONT));
  HFONT old_font=static_cast<HFONT>(SelectObject(hdc, fixed_font));

  SetBkColor(hdc, 0x000000);
  SetTextColor(hdc, 0xffffff);

  if (p->text.size() > 0)
  {
    SetTextAlign(hdc, TA_TOP | TA_LEFT);

    RECT rect;
    GetClientRect(p->hwnd, &rect);

    std::vector<std::string> list;

    // get total size of text and calculate position

    gutil::split(list, p->text, '\n', false);

    int w=0;
    int h=0;

    for (size_t i=0; i<list.size(); i++)
    {
      if (list[i].size() > 0)
      {
        SIZE size;
        GetTextExtentPoint32(hdc, TEXT(list[i].c_str()), static_cast<int>(list[i].size()), &size);
        w=std::max(w, static_cast<int>(size.cx));
      }

      h+=p->th;
    }

    if (rect.right-rect.left > w)
    {
      rect.left+=(rect.right-rect.left-w)/2;
      rect.right=rect.left+w;
    }

    if (rect.bottom-rect.top > h)
    {
      rect.top+=(rect.bottom-rect.top-h)/2;
      rect.bottom=rect.top+h;
    }

    HBRUSH brush=CreateSolidBrush(GetBkColor(hdc));
    FillRect(hdc, &rect, brush);
    DeleteObject(brush);

    // draw multi line text

    for (size_t i=0; i<list.size(); i++)
    {
      if (list[i].size() > 0)
      {
        TextOut(hdc, rect.left, static_cast<int>(rect.top+i*p->th), TEXT(list[i].c_str()),
                static_cast<int>(list[i].size()));
      }
    }
  }
  else if (p->info.size() > 0)
  {
    SetTextAlign(hdc, TA_TOP | TA_LEFT);

    RECT rect;
    GetClientRect(p->hwnd, &rect);

    SIZE size;
    GetTextExtentPoint32(hdc, TEXT(p->info.c_str()), static_cast<int>(p->info.size()), &size);

    // alternatively, draw one line of text at specified position

    if (p->top)
    {
      if (p->left)
      {
        TextOut(hdc, rect.left, rect.top, TEXT(p->info.c_str()), static_cast<int>(p->info.size()));
      }
      else
      {
        TextOut(hdc, rect.right-size.cx, rect.top, TEXT(p->info.c_str()), static_cast<int>(p->info.size()));
      }
    }
    else
    {
      if (p->left)
      {
        TextOut(hdc, rect.left, rect.bottom-size.cy, TEXT(p->info.c_str()),
                static_cast<int>(p->info.size()));
      }
      else
      {
        TextOut(hdc, rect.right-size.cy, rect.bottom-size.cy, TEXT(p->info.c_str()),
                static_cast<int>(p->info.size()));
      }
    }
  }

  if (old_font != 0)
  {
    SelectObject(hdc, old_font);
  }
}

int getState(WPARAM wparam)
{
  int state=0;

  if ((wparam & MK_CONTROL) != 0)
  {
    state|=BaseWindow::ctrlmask;
  }

  if ((wparam & MK_SHIFT) != 0)
  {
    state|=BaseWindow::shiftmask;
  }

  if ((wparam & MK_LBUTTON) != 0)
  {
    state|=BaseWindow::button1mask;
  }

  if ((wparam & MK_MBUTTON) != 0)
  {
    state|=BaseWindow::button2mask;
  }

  if ((wparam & MK_RBUTTON) != 0)
  {
    state|=BaseWindow::button3mask;
  }

  return state;
}

BaseWindow::SpecialKey getSpecialKey(WPARAM wparam)
{
  BaseWindow::SpecialKey sk=BaseWindow::k_none;

  switch (wparam)
  {
    case VK_ESCAPE:
      sk=BaseWindow::k_esc;
      break;

    case VK_LEFT:
      sk=BaseWindow::k_left;
      break;

    case VK_RIGHT:
      sk=BaseWindow::k_right;
      break;

    case VK_UP:
      sk=BaseWindow::k_up;
      break;

    case VK_DOWN:
      sk=BaseWindow::k_down;
      break;

    case VK_HOME:
      sk=BaseWindow::k_home;
      break;

    case VK_END:
      sk=BaseWindow::k_end;
      break;

    default:
      sk=BaseWindow::k_none;
      break;
  }

  return sk;
}

LRESULT CALLBACK mainProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
  try
  {
    switch (msg)
    {
      case WM_GETMINMAXINFO:
        {
          MINMAXINFO *minmax=(MINMAXINFO *) lparam;
          minmax->ptMinTrackSize.x=200;
          minmax->ptMinTrackSize.y=100;

          return 0;
        }

      case WM_CREATE:
        {
          return 0;
        }

      case WM_PAINT:
        {
          PAINTSTRUCT ps;
          HDC hdc=BeginPaint(bp->hwnd, &ps);

          if (hdc != 0)
          {
            HDC hdb=CreateCompatibleDC(NULL);
            HGDIOBJ old=SelectObject(hdb, static_cast<HGDIOBJ>(bp->bitmap));

            BitBlt(hdc, ps.rcPaint.left, ps.rcPaint.top,
                   ps.rcPaint.right-ps.rcPaint.left, ps.rcPaint.bottom-ps.rcPaint.top,
                   hdb, ps.rcPaint.left, ps.rcPaint.top, SRCCOPY);

            drawInfoText(bp->hwnd, hdc, bp);

            SelectObject(hdb, old);
            DeleteDC(hdb);
            EndPaint(bp->hwnd, &ps);
          }

          return 0;
        }

      case WM_SIZE:
        {
          bp->base->onResize(static_cast<int>(LOWORD(lparam)), static_cast<int>(HIWORD(lparam)));
          return 0;
        }

      case WM_LBUTTONDOWN:
        {
          bp->base->onMousePressed(BaseWindow::button1, static_cast<int>(GET_X_LPARAM(lparam)),
                                   static_cast<int>(GET_Y_LPARAM(lparam)), getState(wparam));
          return 0;
        }

      case WM_MBUTTONDOWN:
        {
          bp->base->onMousePressed(BaseWindow::button2, static_cast<int>(GET_X_LPARAM(lparam)),
                                   static_cast<int>(GET_Y_LPARAM(lparam)), getState(wparam));
          return 0;
        }

      case WM_RBUTTONDOWN:
        {
          bp->base->onMousePressed(BaseWindow::button3, static_cast<int>(GET_X_LPARAM(lparam)),
                                   static_cast<int>(GET_Y_LPARAM(lparam)), getState(wparam));
          return 0;
        }

      case WM_LBUTTONUP:
        {
          bp->base->onMouseReleased(BaseWindow::button1, static_cast<int>(GET_X_LPARAM(lparam)),
                                    static_cast<int>(GET_Y_LPARAM(lparam)), getState(wparam));
          return 0;
        }

      case WM_MBUTTONUP:
        {
          bp->base->onMouseReleased(BaseWindow::button2, static_cast<int>(GET_X_LPARAM(lparam)),
                                    static_cast<int>(GET_Y_LPARAM(lparam)), getState(wparam));
          return 0;
        }

      case WM_RBUTTONUP:
        {
          bp->base->onMouseReleased(BaseWindow::button3, static_cast<int>(GET_X_LPARAM(lparam)),
                                    static_cast<int>(GET_Y_LPARAM(lparam)), getState(wparam));
          return 0;
        }

      case WM_MOUSEWHEEL:
        {
          POINT p;

          p.x=GET_X_LPARAM(lparam);
          p.y=GET_Y_LPARAM(lparam);

          ScreenToClient(bp->hwnd, &p);

          if (GET_WHEEL_DELTA_WPARAM(wparam)/120 > 0)
          {
            bp->base->onMousePressed(BaseWindow::button4, static_cast<int>(p.x),
                                     static_cast<int>(p.y), getState(wparam&0xffff));
          }
          else if (GET_WHEEL_DELTA_WPARAM(wparam)/120 < 0)
          {
            bp->base->onMousePressed(BaseWindow::button5, static_cast<int>(p.x),
                                     static_cast<int>(p.y), getState(wparam&0xffff));
          }

          return 0;
        }

      case WM_MOUSEMOVE:
        {
          if ((wparam & (MK_LBUTTON|MK_MBUTTON|MK_RBUTTON)) != 0)
            bp->base->onMouseMove(static_cast<int>(GET_X_LPARAM(lparam)),
                                  static_cast<int>(GET_Y_LPARAM(lparam)), getState(wparam));

          return 0;
        }

      case WM_CHAR:
        {
          int   x=0;
          int   y=0;

          POINT p;

          if (GetCursorPos(&p))
          {
            if (ScreenToClient(bp->hwnd, &p))
            {
              x=static_cast<int>(p.x);
              y=static_cast<int>(p.y);
            }
          }

          bp->base->onKey(static_cast<char>(wparam), BaseWindow::k_none, x, y);

          return 0;
        }

      case WM_KEYDOWN:
        {
          BaseWindow::SpecialKey sk=getSpecialKey(wparam);

          if (sk != BaseWindow::k_none)
          {
            int   x=0;
            int   y=0;

            POINT p;

            if (GetCursorPos(&p))
            {
              if (ScreenToClient(bp->hwnd, &p))
              {
                x=static_cast<int>(p.x);
                y=static_cast<int>(p.y);
              }
            }

            bp->base->onKey(0, getSpecialKey(wparam), x, y);
            return 0;
          }

          break;
        }

      case WM_CLOSE:
        {
          if (bp->base->onClose())
          {
            DestroyWindow(bp->hwnd);
          }

          return 0;
        }

      case WM_DESTROY:
        {
          bp->hwnd=0;
          PostQuitMessage(0);
          return 0;
        }
    }
  }
  catch (const gutil::Exception &ex)
  {
    ex.print();
    exit(10);
  }
  catch (...)
  {
    gutil::showError("An unknown exception occured");
    exit(10);
  }

  return DefWindowProc(hwnd, msg, wparam, lparam);
}

}

BaseWindow::BaseWindow(const char *title, int w, int h)
{
  if (bp != 0)
  {
    throw WindowsException("There can be only one BaseWindow!");
  }

  bp=p=new BaseWindowData();

  p->base=this;
  p->hinstance=GetModuleHandle(NULL);

  // initialise common controls

  INITCOMMONCONTROLSEX icc;

  icc.dwSize = sizeof(icc);
  icc.dwICC = ICC_WIN95_CLASSES;
  InitCommonControlsEx(&icc);

  // class for window

  WNDCLASSEX wc;
  LPCTSTR mainclass=TEXT("sv");
  LPCTSTR maintitle=TEXT(title);

  wc.cbSize=sizeof(wc);
  wc.style=0;
  wc.lpfnWndProc=&mainProc;
  wc.cbClsExtra=0;
  wc.cbWndExtra=0;
  wc.hInstance=p->hinstance;
  wc.hIcon=0;
  wc.hCursor=(HCURSOR) LoadImage(NULL, IDC_ARROW, IMAGE_CURSOR, 0, 0,
                                 LR_SHARED);
  wc.hbrBackground=0;
  wc.lpszMenuName=0;
  wc.lpszClassName=mainclass;
  wc.hIconSm=0;

  // register window class

  if (!RegisterClassEx(&wc))
  {
    MessageBox(NULL, TEXT("Error registering window class"), TEXT("Error"), MB_ICONERROR | MB_OK);
    throw WindowsException("Error registering window class");
  }

  // create instance of window

  p->hwnd=CreateWindowEx(0, mainclass, maintitle, WS_OVERLAPPEDWINDOW,
                         CW_USEDEFAULT, CW_USEDEFAULT, w, h, NULL, NULL, p->hinstance, NULL);

  // error if window creation failed

  if (p->hwnd == 0)
  {
    MessageBox(NULL, TEXT("Error creating main window"), TEXT("Error"), MB_ICONERROR | MB_OK);
    throw WindowsException("Error creating main window");
  }

  // get size of window borders

  RECT rect;
  rect.left=50;
  rect.top=50;
  rect.right=350;
  rect.bottom=250;

  AdjustWindowRectEx(&rect, WS_OVERLAPPEDWINDOW, FALSE, 0);

  p->xb=(rect.right-rect.left-300);
  p->yb=(rect.bottom-rect.top-200);

  // get height of text

  GetClientRect(GetDesktopWindow(), &rect);

  HDC hdc=GetDC(p->hwnd);

  HFONT fixed_font=static_cast<HFONT>(GetStockObject(ANSI_FIXED_FONT));
  HFONT old_font=static_cast<HFONT>(SelectObject(hdc, fixed_font));

  SIZE size;
  GetTextExtentPoint32(hdc, TEXT("ABCDEFG"), 7, &size);
  p->th=size.cy;

  if (old_font != 0)
  {
    SelectObject(hdc, old_font);
  }

  // create memory bitmap as buffer

  p->bw=rect.right;
  p->bh=rect.bottom;
  p->bytes_per_line=(3*p->bw+3)&~3;

  BITMAPINFO info;
  info.bmiHeader.biSize=sizeof(BITMAPINFOHEADER);
  info.bmiHeader.biWidth=p->bw;
  info.bmiHeader.biHeight=p->bh;
  info.bmiHeader.biPlanes=1;
  info.bmiHeader.biBitCount=24;
  info.bmiHeader.biCompression=BI_RGB;
  info.bmiHeader.biSizeImage=0;
  info.bmiHeader.biXPelsPerMeter=1;
  info.bmiHeader.biYPelsPerMeter=1;
  info.bmiHeader.biClrUsed=0;
  info.bmiHeader.biClrImportant=0;

  p->bitmap=CreateDIBSection(hdc, &info, DIB_RGB_COLORS,
                             reinterpret_cast<void **>(&(p->bpixel)), 0, 0);

  if (p->bitmap == 0)
  {
    MessageBox(NULL, TEXT("Cannot create bitmap"), TEXT("Error"), MB_ICONERROR | MB_OK);
    throw WindowsException("Cannot create bitmap");
  }

  ReleaseDC(p->hwnd, hdc);
}

BaseWindow::~BaseWindow()
{
  bp=0;

  DeleteObject(p->bitmap);

  if (p->hwnd != 0)
  {
    DestroyWindow(p->hwnd);
  }

  delete p;
}

void BaseWindow::setIcon(const gimage::ImageU8 &icon)
{
  // not yet implemented for Windows
}

void BaseWindow::setVisible(bool show)
{
  if (p->hwnd != 0)
  {
    if (show)
    {
      ShowWindow(p->hwnd, SW_SHOW);
      UpdateWindow(p->hwnd);
    }
    else
    {
      ShowWindow(p->hwnd, SW_HIDE);
    }
  }
}

int BaseWindow::addFileWatch(const char *path)
{
  // how to check if a file has been modified and the modification
  // is completed, i.e. writing has finished (!) on windows?

  return 0;
}

void BaseWindow::removeFileWatch(int watchid)
{ }

void BaseWindow::sendClose()
{
  if (p->hwnd != 0)
  {
    CallWindowProc(mainProc, p->hwnd, WM_CLOSE, 0, 0);
  }
}

void BaseWindow::waitForClose()
{
  MSG msg;

  while (GetMessage(&msg, NULL, 0, 0) > 0)
  {
    TranslateMessage(&msg);
    DispatchMessage(&msg);
  }
}

bool BaseWindow::isClosed()
{
  return p->hwnd == 0;
}

void BaseWindow::getContent(gimage::ImageU8 &image)
{
  int w=0, h=0;

  getSize(w, h);

  image.setSize(w, h, 3);

  for (int k=0; k<h; k++)
  {
    unsigned char *line=p->bpixel+(p->bh-k-1)*p->bytes_per_line;

    for (int i=0; i<w; i++)
    {
      image.set(i, k, 2, *line++);
      image.set(i, k, 1, *line++);
      image.set(i, k, 0, *line++);
    }
  }
}

void BaseWindow::setTitle(const char *title)
{
  if (p->hwnd != 0)
  {
    SetWindowText(p->hwnd, TEXT(title));
  }
}

void BaseWindow::getDisplaySize(int &w, int &h)
{
  RECT rect;

  // get desktop size

  GetClientRect(GetDesktopWindow(), &rect);
  w=rect.right-rect.left-p->xb;
  h=rect.bottom-rect.top-p->yb;
}

void BaseWindow::getSize(int &w, int &h)
{
  if (p->hwnd != 0)
  {
    RECT rect;

    // get size of window without borders

    GetClientRect(p->hwnd, &rect);
    w=rect.right-rect.left;
    h=rect.bottom-rect.top;
  }
}

void BaseWindow::setSize(int w, int h)
{
  if (p->hwnd != 0)
  {
    RECT rect;
    int  x, y;

    // get current position

    GetWindowRect(p->hwnd, &rect);
    x=rect.left;
    y=rect.top;

    // add window borders

    w+=p->xb;
    h+=p->yb;

    // get size of desktop, check that the window is not made bigger
    // than the desktop and also check if the window should be moved

    GetClientRect(GetDesktopWindow(), &rect);

    w=std::min(w, static_cast<int>(rect.right-rect.left));
    h=std::min(h, static_cast<int>(rect.bottom-rect.top));

    if (x+w > rect.right)
    {
      x=rect.right-w;

      if (x < 0)
      {
        x=0;
      }
    }

    if (y+h > rect.bottom)
    {
      y=rect.bottom-h;

      if (y < 0)
      {
        y=0;
      }
    }

    SetWindowPos(p->hwnd, HWND_TOP, x, y, w, h, SWP_NOZORDER);
  }
}

void BaseWindow::setPosition(int x, int y)
{
  if (p->hwnd != 0)
  {
    x=std::min(x, 0);
    y=std::min(y, 0);

    RECT drect;
    GetClientRect(GetDesktopWindow(), &drect);

    RECT wrect;
    GetWindowRect(GetDesktopWindow(), &wrect);

    x=std::min(x, static_cast<int>(drect.right-(wrect.right-wrect.left)));
    y=std::min(y, static_cast<int>(drect.bottom-(wrect.bottom-wrect.top)));

    if (p->hwnd != 0)
    {
      SetWindowPos(p->hwnd, HWND_TOP, x, y, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
    }
  }
}

int BaseWindow::getTextHeight()
{
  return p->th;
}

void BaseWindow::setInfoLine(const char *text, bool top, bool left)
{
  if (strlen(text) == 0 && p->info.size() == 0)
  {
    return;
  }

  if (p->hwnd == 0)
  {
    p->info=text;
    p->top=top;
    p->left=left;
    return;
  }

  // get context for window and bitmap

  HDC hdc=GetDC(p->hwnd);
  HDC hdb=CreateCompatibleDC(NULL);
  HGDIOBJ old=SelectObject(hdb, static_cast<HGDIOBJ>(p->bitmap));

  // get inner size of window

  RECT rect;
  GetClientRect(p->hwnd, &rect);

  // determine size of old info text

  SIZE size;
  size.cx=0;
  size.cy=0;

  if (p->info.size() > 0)
  {
    GetTextExtentPoint32(hdc, TEXT(p->info.c_str()), static_cast<int>(p->info.size()), &size);
  }

  // clear old std::string completely, if position has changed

  if (p->info.size() > 0 && (top != p->top || left != p->left))
  {
    if (p->top)
    {
      if (p->left)
        BitBlt(hdc, rect.left, rect.top, size.cx, size.cy, hdb,
               rect.left, rect.top, SRCCOPY);
      else
        BitBlt(hdc, rect.right-size.cx, rect.top, size.cx, size.cy, hdb,
               rect.right-size.cx, rect.top, SRCCOPY);
    }
    else
    {
      if (p->left)
        BitBlt(hdc, rect.left, rect.bottom-size.cy, size.cx, size.cy, hdb,
               rect.left, rect.bottom-size.cy, SRCCOPY);
      else
        BitBlt(hdc, rect.right-size.cx, rect.bottom-size.cy, size.cx, size.cy, hdb,
               rect.right-size.cx, rect.bottom-size.cy, SRCCOPY);
    }

    size.cx=0;
    size.cy=0;
  }

  // store and draw new info text

  p->info=text;
  p->top=top;
  p->left=left;

  drawInfoText(p->hwnd, hdc, p);

  // clear overlapping parts of old info text

  if (size.cx > 0 && size.cy > 0)
  {
    SIZE nsize;
    GetTextExtentPoint32(hdc, TEXT(p->info.c_str()), static_cast<int>(p->info.size()), &nsize);

    size.cx-=nsize.cx;

    if (size.cx > 0)
    {
      if (p->top)
      {
        if (p->left)
          BitBlt(hdc, rect.left+nsize.cx, rect.top, size.cx, size.cy, hdb,
                 rect.left+nsize.cx, rect.top, SRCCOPY);
        else
          BitBlt(hdc, rect.right-nsize.cx-size.cx, rect.top, size.cx, size.cy, hdb,
                 rect.right-nsize.cx-size.cx, rect.top, SRCCOPY);
      }
      else
      {
        if (p->left)
          BitBlt(hdc, rect.left+nsize.cx, rect.bottom-p->th, size.cx, size.cy, hdb,
                 rect.left+nsize.cx, rect.bottom-p->th, SRCCOPY);
        else
          BitBlt(hdc, rect.right-nsize.cx-size.cx, rect.bottom-p->th, size.cx, size.cy, hdb,
                 rect.right-nsize.cx-size.cx, rect.bottom-p->th, SRCCOPY);
      }
    }
  }

  // free resources

  SelectObject(hdb, old);
  DeleteDC(hdb);
  ReleaseDC(p->hwnd, hdc);
}

void BaseWindow::setInfoText(const char *text)
{
  if (strlen(text) == 0 && p->text.size() == 0)
  {
    return;
  }

  if (p->hwnd == 0)
  {
    p->text=text;
    return;
  }

  // get context for window and bitmap

  HDC hdc=GetDC(p->hwnd);
  HDC hdb=CreateCompatibleDC(NULL);
  HGDIOBJ old=SelectObject(hdb, static_cast<HGDIOBJ>(bp->bitmap));

  // get inner size of window

  RECT rect;
  GetClientRect(p->hwnd, &rect);

  // clear completely

  BitBlt(hdc, rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top, hdb,
         rect.left, rect.top, SRCCOPY);

  // store text and draw

  p->text=text;
  drawInfoText(p->hwnd, hdc, p);

  // free resources

  SelectObject(hdb, old);
  DeleteDC(hdb);
  ReleaseDC(p->hwnd, hdc);
}

bool BaseWindow::hasInfoText()
{
  return p->text.size() != 0;
}

void BaseWindow::clearBuffer()
{
  memset(p->bpixel, 0, p->bytes_per_line*p->bh);
}

void BaseWindow::paintBuffer(const ImageAdapterBase &im, int x, int y)
{
  const int w=std::min(static_cast<long>(p->bw), x+im.getWidth());
  const int h=std::min(static_cast<long>(p->bh), y+im.getHeight());

  for (int k=std::max(0, y); k<h; k++)
  {
    gimage::ImageU8 buffer(w, 1, 3);
    im.copyInto(buffer, -x , k-y);

    unsigned char *line=p->bpixel+(p->bh-k-1)*p->bytes_per_line+3*std::max(0, x);

    for (int i=std::max(0, x); i<w; i++)
    {
      *line++=buffer.get(i, 0, 2);
      *line++=buffer.get(i, 0, 1);
      *line++=buffer.get(i, 0, 0);
    }
  }
}

void BaseWindow::showBuffer()
{
  if (p->hwnd != 0)
  {
    RedrawWindow(p->hwnd, 0, 0, RDW_INVALIDATE | RDW_UPDATENOW);  // or RDW_ERASENOW?
  }
}

}
