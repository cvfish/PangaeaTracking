#include <GL/glew.h>

#include "main_engine/rendering/NcvGlXContext.h"

namespace ncv
{

    // Return true if GL error occured, print out error with message msg
	bool checkGlError( string msg )
	{
		GLenum error = GL_NO_ERROR;

		error = glGetError();

		if (error != GL_NO_ERROR)
		{
			cerr << msg << endl;
			cerr << "\t(" << gluErrorString(error) << ")" << endl;

			return true;
		}

		return false;
	}
    
	GlXOffscreenContext::GlXOffscreenContext(uint width, uint height) :
		_display(NULL),
		_fbConfig(None),
		_pBuffer(0),
		_context(NULL),
		_width(width),
		_height(height),
		_ready(false)
	{
		if ((_width == 0) || (_height == 0))
			localError("Both width and height must be greater than zero.");

		createContext();
	}

	GlXOffscreenContext::~GlXOffscreenContext()
	{
		clearActive();

		_ready = false;

		glXDestroyContext(_display, _context);
		glXDestroyPbuffer(_display, _pBuffer);

		XCloseDisplay(_display);
	}

	bool GlXOffscreenContext::makeActive()
	{
		if (!isReady())
			return false;

		if (isActive())
			return true;

		if (!glXMakeContextCurrent(_display, _pBuffer, _pBuffer, _context))
		{
			localError("Unable to make context active.");
			return false;
		}
		else
			return true;
	}

	void GlXOffscreenContext::clearActive()
	{
		if (!isReady())
			return;

		if (!glXMakeCurrent(_display, None, NULL))
			localError("Unable to release context.");
	}

	int GlXOffscreenContext::getIntProperty(GLenum intProperty)
	{
		ContextRestorer restorer(!isActive());
		makeActive();

		int v = 0;
		glGetIntegerv(intProperty, &v);
		checkGlError("After GlXOffscreenContext::getIntProperty..");
		return v;
	}

	void GlXOffscreenContext::createContext()
	{
		_ready = false;

		_display = XOpenDisplay(NULL);

		if (!_display)
			localError("Unable to open XDisplay.");

		int errorBase, eventBase;
		if (!glXQueryExtension(_display, &errorBase, &eventBase))
			localError("OpenGL GLX Extension not supported.");

		int screen = DefaultScreen(_display);

		int glxVersionMajor, glxVersionMinor;
		glXQueryVersion(_display, &glxVersionMajor, &glxVersionMinor);

		dbg << "GLX Version: " << glxVersionMajor << "." << glxVersionMinor << endl;

		int fbConfigCount = 0;

		int attribList[] =
		{
			//GLX_X_RENDERABLE, True, // Implied
			GLX_DRAWABLE_TYPE, GLX_PBUFFER_BIT,// | GLX_WINDOW_BIT,
			GLX_RENDER_TYPE, GLX_RGBA_BIT,
			GLX_RED_SIZE, 8,
			GLX_BLUE_SIZE, 8,
			GLX_GREEN_SIZE, 8,
			//GLX_ALPHA_SIZE, 8,
			//GLX_MAX_PBUFFER_WIDTH, _width,
			//GLX_MAX_PBUFFER_HEIGHT, _height,
			GLX_DEPTH_SIZE, 24,
			GLX_DOUBLEBUFFER, False,
			None
		};

		GLXFBConfig* fbConfigs = glXChooseFBConfig(_display, screen, attribList, &fbConfigCount);

		if ((fbConfigs == NULL) || (fbConfigCount <= 0))
			localError("Unable to choose fb config.");

		int pBufferAttribList[] =
		{
			GLX_PBUFFER_WIDTH, (int)_width,
			GLX_PBUFFER_HEIGHT, (int)_height,
			GLX_LARGEST_PBUFFER, False,
			None
		};

		for (int i = 0; i < fbConfigCount; ++i)
		{
			_pBuffer = glXCreatePbuffer(_display, fbConfigs[i], pBufferAttribList);

			if (_pBuffer != None)
			{
				dbg << "Found successful frame buffer on attempt " << (i+1) << ".." << endl;
				_fbConfig = fbConfigs[i];
				break;
			}
		}

		XSync(_display, False);

		XFree(fbConfigs);

		if (_pBuffer == None)
			localError("Unable to create pbuffer.");

		// Get visual..
		XVisualInfo* visInfo = glXGetVisualFromFBConfig(_display, _fbConfig);
		if (!visInfo)
		{
			localError("Unable to get X visual.");
		}

		// Create context..
		_context = glXCreateContext(_display, visInfo, NULL, True);

		if (!_context)
		{
			dbg << "Unable to create a direct context, attempting indirect one.." << endl;
			_context = glXCreateContext(_display, visInfo, NULL, False);
		}
		else
		{
			dbg << "Created a direct context.." << endl;
		}

		XFree(visInfo);

		if (!_context)
			localError("Unable to create GL context.");

		if (!glXMakeContextCurrent(_display, _pBuffer, _pBuffer, _context))
			localError("Unable to make context active.");

		// Should be able to use openGL here..

		GLenum glewErr = glewInit();
		if (glewErr != GLEW_OK)
		{
			cerr << "GLEW ERROR: " << glewGetErrorString(glewErr) << endl;
		}

		int majorVersion, minorVersion;
		glGetIntegerv(GL_MAJOR_VERSION, &majorVersion);
		glGetIntegerv(GL_MINOR_VERSION, &minorVersion);

		dbg << "GL Version: " << majorVersion << "." << minorVersion << endl;

		checkGlError("After context creation..");

		// Yay
		_ready = true;
	}
}
