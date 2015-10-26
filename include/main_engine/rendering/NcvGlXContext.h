#ifndef NCVGLXCONTEXT_H
#define NCVGLXCONTEXT_H

#include "../utils/global.h"

#include <GL/glew.h>
#include <GL/glx.h>

#include <X11/X.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>

namespace ncv
{
	class GlXOffscreenContext
	{
	public:

		GlXOffscreenContext(uint width, uint height);

		~GlXOffscreenContext();

		inline bool isReady() const
		{
			return _ready;
		}

		bool makeActive();

		void clearActive();

		inline bool isExtensionSupported(const string extensionStr)
		{
			ContextRestorer restorer(!isActive());
			makeActive();
			return glewIsSupported(extensionStr.c_str());
		}

		int getIntProperty(GLenum intProperty);

		inline bool isActive()
		{
			if (!isReady())
				return false;

			GLXContext currContext = glXGetCurrentContext();

			return (currContext == _context);
		}

		inline static bool someContextIsActive()
		{
			return (glXGetCurrentContext() != NULL);
		}

		class ContextRestorer
		{
		public:
			explicit ContextRestorer(bool shouldRestore = true)
			{
				reset();

				_displayToRestore = glXGetCurrentDisplay();
				_drawableToRestore = glXGetCurrentDrawable();
				_contextToRestore = glXGetCurrentContext();
			}

			~ContextRestorer()
			{
				restore();
			}

			void restore()
			{
				if (_shouldRestore)
				{
					if ((_displayToRestore) && (_contextToRestore) &&
							(_drawableToRestore != None))
					{
						if (!glXMakeCurrent(_displayToRestore, _drawableToRestore,
											_contextToRestore))
						{
							cerr << "ERROR: GlXContext: ContextRestorer: "
									"Unable to make context active." << endl;
						}

						reset();
					}
				}
			}

		private:

			void reset()
			{
				_shouldRestore = false;
				_displayToRestore = NULL;
				_drawableToRestore = None;
				_contextToRestore = NULL;
			}

			bool _shouldRestore;
			Display* _displayToRestore;
			GLXDrawable _drawableToRestore;
			GLXContext _contextToRestore;
		};

	private:

		void createContext();

		inline void localError(const char* msg)
		{
			string errorString = string("ERROR: GlXContext: ") + string(msg);
			throw runtime_error(errorString.c_str());
		}

	private:
		Display* _display;
		GLXFBConfig _fbConfig;
		GLXPbuffer _pBuffer;
		GLXContext _context;

		uint _width;
		uint _height;

		bool _ready;
	};

	typedef boost::shared_ptr<GlXOffscreenContext> GlXOffscreenContextPtr;
}

#endif // NCVGLXCONTEXT_H
