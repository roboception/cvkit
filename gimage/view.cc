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

#include "view.h"
#include "arithmetic.h"
#include "io.h"

#include <gutil/misc.h>

#include <iostream>

namespace gimage
{

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::set;

using gutil::split;
using gutil::Properties;
using gutil::Exception;
using gutil::IOException;
using gutil::getFileList;

using gmath::Vector3d;
using gmath::Matrix33d;
using gmath::Camera;
using gmath::PinholeCamera;
using gmath::OrthoCamera;

View::View()
{
    camera=0;
}

View::View(const View &v)
{
    image=v.image;
    depth=v.depth;
    camera=v.getCamera()->clone();
}

View::~View()
{
    delete camera;
}

const View &View::operator=(const View &v)
{
    delete camera;
    
    image=v.image;
    depth=v.depth;
    camera=v.getCamera()->clone();
    
    return *this;
}

void View::clear()
{
    delete camera;
    
    image.setSize(0, 0, 0);
    depth.setSize(0, 0, 0);
    camera=0;
}

void View::setCamera(const Camera *c)
{
    if (c == 0)
    {
      camera=0;
      return;
    }
    
    if (c->getWidth() > 0 && c->getHeight() > 0)
    {
        // make sure that the size of the camera is the same as the size of the
        // images
      
      if (image.getWidth() > 0 && image.getHeight() > 0)
      {
        assert(c->getWidth() == image.getWidth());
        assert(c->getHeight() == image.getHeight());
      }
      
      if (depth.getWidth() > 0 && depth.getHeight() > 0)
      {
        assert(c->getWidth() == depth.getWidth());
        assert(c->getHeight() == depth.getHeight());
      }
      
      camera=c->clone();
    }
    else
    {
      camera=c->clone();
      
        // set size of camera from images if neccessary
      
      if (image.getWidth() > 0 && image.getHeight() > 0)
        camera->setSize(image.getWidth(), image.getHeight());
      
      if (depth.getWidth() > 0 && depth.getHeight() > 0)
        camera->setSize(depth.getWidth(), depth.getHeight());
    }
}

void View::setImage(const ImageU8 &img)
{
      // make sure that the size of the camera is the same as the size of the
      // images
    
    if (camera != 0 && camera->getWidth() > 0 && camera->getHeight() > 0)
    {
      assert(camera->getWidth() == img.getWidth());
      assert(camera->getHeight() == img.getHeight());
    }
    
    if (depth.getWidth() > 0 && depth.getHeight() > 0)
    {
      assert(img.getWidth() == depth.getWidth());
      assert(img.getHeight() == depth.getHeight());
    }
    
    image=img;
    
      // set size of camera from images if neccessary
    
    if (camera != 0 && camera->getWidth() == 0 && camera->getHeight() == 0)
      camera->setSize(image.getWidth(), image.getHeight());
}

void View::setDepthImage(const ImageFloat &d)
{
      // make sure that the size of the camera is the same as the size of the
      // images
    
    if (camera != 0 && camera->getWidth() > 0 && camera->getHeight() > 0)
    {
      assert(camera->getWidth() == d.getWidth());
      assert(camera->getHeight() == d.getHeight());
    }
    
    if (image.getWidth() > 0 && image.getHeight() > 0)
    {
      assert(d.getWidth() == image.getWidth());
      assert(d.getHeight() == image.getHeight());
    }
    
    depth=d;
    
      // set size of camera from images if neccessary
    
    if (camera != 0 && camera->getWidth() == 0 && camera->getHeight() == 0)
      camera->setSize(depth.getWidth(), depth.getHeight());
}

void loadView(View &view, const char *name, const char *spath, bool verbose)
{
    view.clear();
    
      // get optionally specified downscale factor and image part
    
    int  ds=1;
    long x=0, y=0, w=-1, h=-1;
    vector<string> list;
    
    split(list, name, ',');
    
    for (size_t i=1; i<list.size(); i++)
    {
      if (list[i].compare(0, 3, "ds=") == 0)
        ds=atol(list[i].substr(3).c_str());
      
      if (list[i].compare(0, 2, "x=") == 0)
        x=atol(list[i].substr(2).c_str());
      
      if (list[i].compare(0, 2, "y=") == 0)
        y=atol(list[i].substr(2).c_str());
      
      if (list[i].compare(0, 2, "w=") == 0)
        w=atol(list[i].substr(2).c_str());
      
      if (list[i].compare(0, 2, "h=") == 0)
        h=atol(list[i].substr(2).c_str());
    }
    
      // load camera
    
    Camera *camera=0;
    Properties prop;
    
    loadViewProperties(prop, name, spath, verbose);
    
    try
    {
      camera=new PinholeCamera(prop);
    }
    catch (Exception &ex)
    {
      try
      {
        camera=new OrthoCamera(prop);
      }
      catch (Exception &ex2)
      {
        if (verbose)
        {
          cout << "Cannot create pinhole camera from properties because: " << ex.what() << endl;
          cout << "Cannot create ortho camera from properties because: " << ex2.what() << endl;
        }
        
        throw IOException("Cannot create camera object from properties");
      }
    }
    
      // set downscale factor and select part
    
    camera->setDownscaled(ds);
    
    if (w <= 0 && camera->getWidth() > 0)
      w-=x;
    
    if (h <= 0 && camera->getHeight() > 0)
      h-=y;
    
    camera->setPart(x, y, w, h);
    
    view.setCamera(camera);
    
    delete camera;
    camera=0;
    
      // load disparity
    
    ImageFloat depth;
    getImageIO().load(depth, list[0].c_str(), ds, x, y, w, h);
    
      // consider optional offset
    
    float doffs;
    prop.getValue("doffs", doffs, "0");
    
    if (doffs != 0)
      depth+=doffs;
    
      // optionally downscale and set depth image
    
    if (ds > 1)
      depth/=ds;
    
    view.setDepthImage(depth);
    depth.setSize(0, 0, 0);
    
      // load image
    
    string imagename;
    getViewImageName(imagename, name, spath, verbose);
    
    if (imagename.size() > 0)
    {
      ImageU8 image;
      
      try
      {
        getImageIO().load(image, imagename.c_str(), ds, x, y, w, h);
        view.setImage(image);
      }
      catch (Exception &ex)
      {
          // load 16 bit image and convert to 8 bit
        
        try
        {
          ImageU16 image2;
          getImageIO().load(image2, imagename.c_str(), ds, x, y, w, h);
          
          double s=image2.maxValue()/255.0;
          
          if (s != 0)
            image2/=s;
          
          image.setImageLimited(image2);
          image2.setSize(0, 0, 0);
          
          view.setImage(image);
        }
        catch (Exception &ex)
        {
          if (verbose)
            cout << "Cannot load image: " << imagename << " (" << ex.what() << ")" << endl;
        }
      }
    }
}

void getPrefixAlternatives(vector<string> &list, const string &depthname,
  const char *spath)
{
      // determine possible prefixes
    
    size_t spos, pos;
    
    spos=depthname.find_last_of("/\\");
    
    if (spos == depthname.npos)
      spos=0;
    else
      spos++;
    
    pos=depthname.rfind(':');
    if (pos != depthname.npos && depthname.compare(pos, 2, ":\\") == 0)
      pos=depthname.npos;
    
    if (pos == depthname.npos)
    {
      pos=depthname.rfind('.');
      
      if (pos == depthname.npos || pos < spos)
        return;
    }
    
    list.push_back(depthname.substr(0, pos));
    
    pos=depthname.rfind('_', pos-1);
    while (pos != depthname.npos && pos > spos)
    {
      list.push_back(depthname.substr(0, pos));
      pos=depthname.rfind('_', pos-1);
    }
    
    int n=list.size();
    
      // combine possible prefixes with all directories of the optional search
      // path
    
    if (spath != 0 && spath[0] != '\0')
    {
      vector<string> dir;
      
#ifdef WIN32
      split(dir, spath, ';');
#else
      split(dir, spath, ':');
#endif
      
      for (size_t i=0; i<dir.size(); i++)
      {
#ifdef WIN32
        if (dir[i][dir[i].size()-1] != '\\')
          dir[i].push_back('\\');
#else
        if (dir[i][dir[i].size()-1] != '/')
          dir[i].push_back('/');
#endif
        for (int k=0; k<n; k++)
          list.push_back(dir[i]+list[k].substr(spos));
      }
    }
}

void loadViewProperties(Properties &prop, const char *name, const char *spath,
  bool verbose)
{
    vector<string> list;
    
      // load parameter files as specified in name
    
    split(list, name, ',');
    
    bool expl=false;
    for (size_t i=1; i<list.size(); i++)
    {
      if (list[i].compare(0, 2, "p=") == 0)
      {
        prop.load(list[i].substr(2).c_str());
        expl=true;
      }
    }
    
      // alternatively, search for parameter files
    
    if (!expl)
    {
      string depthname=list[0];
      const char *suffix[]={".txt", ".TXT", "_param.txt", "_PARAM.TXT"};
      
      list.clear();
      getPrefixAlternatives(list, depthname, spath);
      for (int i=list.size()-1; i>=0; i--)
      {
        for (int k=0; k<4; k++)
        {
            // try all possibilities and add parameters
          
          try
          {
            prop.load((list[i]+suffix[k]).c_str());
            
            if (verbose)
              cout << "Using parameter file: " << list[i]+suffix[k] << endl;
          }
          catch (const Exception &ex)
          { }
        }
      }
      
        // support format of Middlebury stereo benchmark data sets
      
      size_t pos;
      
      pos=depthname.find_last_of("/\\");
      
      if (pos == depthname.npos)
        pos=0;
      else
        pos++;
      
      if (depthname.size()-pos >= 9 && depthname.substr(pos, 4) == "disp")
      {
        try
        {
          string s=depthname.substr(0, pos)+"calib.txt";
          Properties mprop;
          mprop.load(s.c_str());
          
          if (verbose)
            cout << "Using Middlebury calibration file: " << s << endl;
          
          mprop.getString("width", s, "0");
          prop.putString("camera.width", s);
          mprop.getString("height", s, "0");
          prop.putString("camera.height", s);
          
          Matrix33d A;
          mprop.getValue((string("cam")+depthname[pos+4]).c_str(), A);
          prop.putValue("camera.A", A);
          
          prop.putValue("f", A(0, 0));
          
          double t;
          mprop.getValue("baseline", t);
          prop.putValue("t", t);
          
          mprop.getString("doffs", s, "0");
          prop.putString("doffs", s);
          
          prop.putString("camera.R", "[1 0 0; 0 1 0; 0 0 1]");
          
          Vector3d T;
          T[0]=t*(depthname[pos+4]-'0');
          prop.putValue("camera.T", T);
        }
        catch (const IOException &ex)
        { }
        catch (const Exception &ex)
        {
          cerr << ex.what() << endl;
        }
      }
    }
}

void getViewImageName(string &image, const char *name, const char *spath,
  bool verbose)
{
    vector<string> list;
    
      // get image name from specification
    
    split(list, name, ',');
    
    image.clear();
    for (size_t i=1; i<list.size(); i++)
    {
      if (list[i].compare(0, 2, "i=") == 0)
        image=list[i].substr(2);
    }
    
      // support format of Middlebury stereo benchmark data sets
    
    if (image.size() == 0)
    {
      string depthname=list[0];
      size_t pos;
      
      pos=depthname.find_last_of("/\\");
      
      if (pos == depthname.npos)
        pos=0;
      else
        pos++;
      
      if (depthname.size()-pos >= 9 && depthname.substr(pos, 4) == "disp")
      {
        long dw, dh;
        int  dd;
        
        getImageIO().loadHeader(depthname.c_str(), dw, dh, dd);
        dd=0;
        
          // get list of all files with that prefix
        
        set<string> flist;
        
        try
        {
          getFileList(flist, depthname.substr(0, pos)+"im"+depthname[pos+4]+".", "");
        }
        catch (IOException &ex)
        { }
        
          // check list of files for images with the same size as the depth
          // image and prefer color images
        
        for (set<string>::iterator it=flist.begin(); it!=flist.end(); it++)
        {
          string s=*it;
          
            // try to load image header and compare size to depth image
          
          try
          {
            long w, h;
            int  d;
            
            getImageIO().loadHeader(s.c_str(), w, h, d);
            
            if (w == dw && h == dh)
            {
              if (verbose)
                cout << "Found suitable image: " << s << endl;
              
              if (dd != 3)
              {
                image=s;
                dd=d;
              }
            }
          }
          catch (Exception ex)
          { }
        }
      }
    }
    
      // alternatively, search for image name
    
    if (image.size() == 0)
    {
      string depthname=list[0];
      bool tiled=false;
      
      size_t pos=depthname.rfind(':');
      if (pos != depthname.npos && depthname.compare(pos, 2, ":\\") != 0)
        tiled=true;
      
      long dw, dh;
      int  dd;
      
      getImageIO().loadHeader(depthname.c_str(), dw, dh, dd);
      
      dd=0;
      
        // go through list of prefixes
      
      list.clear();
      getPrefixAlternatives(list, depthname, spath);
      for (size_t i=0; i<list.size(); i++)
      {
          // get list of all files with that prefix
        
        set<string> flist;
        
        try
        {
          getFileList(flist, list[i], "");
        }
        catch (IOException &ex)
        { }
        
          // check list of files for images with the same size as the depth
          // image and prefer color images
        
        set<string> checked;
        checked.insert(depthname);
        
        for (set<string>::iterator it=flist.begin(); it!=flist.end(); it++)
        {
          string s=*it;
          
            // if the depth image is a tiled image, then try to detect the
            // corresponding image as tiled
          
          if (tiled)
          {
            size_t pos=s.find_last_of("/\\");
            
            if (pos == s.npos)
              pos=0;
            
            pos=s.find('_', pos);
            
            if (pos != s.npos)
            {
              size_t pos2=s.rfind('_');
              
              if (pos2 > pos)
              {
                s=s.substr(0, pos)+":"+s.substr(pos2+1);
              }
            }
          }
          
            // try to load image header and compare size to depth image
          
          if (checked.find(s) == checked.end() &&
            s.compare(s.size()-4, 4, ".txt") != 0 &&
            s.compare(s.size()-4, 4, ".TXT") != 0)
          {
            checked.insert(s);
            
            try
            {
              long w, h;
              int  d;
              
              getImageIO().loadHeader(s.c_str(), w, h, d);
              
              if (w == dw && h == dh)
              {
                if (verbose)
                  cout << "Found suitable image: " << s << endl;
                
                if (dd != 3)
                {
                  image=s;
                  dd=d;
                }
              }
            }
            catch (Exception ex)
            { }
          }
        }
      }
    }
    
    if (verbose)
    {
      if (image.size() > 0)
        cout << "Using texture image: " << image << endl;
      else
        cout << "No texture image found" << endl;
    }
}

}
