/*----------------------------------------------------------------------------

3D Physics Demo By Carlos Pegar

*/

#include <windows.h>
#include <windowsx.h>
#include <commdlg.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "winInit.h"

float Zoom = -16.0f;
float dZoom = 0.0f;
float viewAngle = 0.0f;
float dviewAngle = 0.0f;
float height = 0.0f;
float dheight = 0.0f;

#pragma warning (disable:4244) // conversion from float to int

//TODO: add mouse control

int unsigned WindowWidth = 800;
int unsigned WindowHeight = 600;

char szAppName[128] = "3D Physics Demo";

void AppExit(void);
BOOL AppIdle(void);
void Reset( void );
void Initialize( void );
void Execute( float zoom,  float height, float angle );
void Destroy( void );
void ToggleGravity( void );
void ToggleFriction( void );
void selectObjectAtPoint( int xpos, int ypos);


HINSTANCE hInstApp;
HWND      hwndApp;
HPALETTE  hpalApp;
BOOL fAppActive;

HDC WindowDC;
HGLRC WindowRC;

LONG FAR PASCAL AppWndProc(HWND hwnd,UINT msg,WPARAM wParam,LPARAM lParam);
LONG AppCommand (HWND hwnd,UINT msg,WPARAM wParam,LPARAM lParam);
int AppPaint (HWND hwnd, HDC hdc);


BOOL FAR PASCAL  AppAbout(HWND hwnd,UINT msg,WPARAM wParam,LPARAM )
{
    switch (msg)
    {
        case WM_COMMAND:
            if (LOWORD(wParam) == IDOK)
            {
                EndDialog(hwnd,TRUE);
            }
            break;

        case WM_INITDIALOG:
            return TRUE;
    }
    return FALSE;
}

BOOL AppInit(HINSTANCE hInst,HINSTANCE hPrev,int sw,LPSTR szCmdLine)
{

    WNDCLASS cls;

    /* Save instance handle for DialogBoxs */
    hInstApp = hInst;

    if (!hPrev)
    {
        /*
         *  Register a class for the main application window
         */
        cls.hCursor        = LoadCursor(NULL,IDC_ARROW);
        cls.hIcon          = LoadIcon(hInst,"AppIcon");
        cls.lpszMenuName   = "AppMenu";
        cls.lpszClassName  = szAppName;
        cls.hbrBackground  = (HBRUSH)(COLOR_WINDOW + 1);
        cls.hInstance      = hInst;
        cls.style          = CS_BYTEALIGNCLIENT |
                                CS_VREDRAW | CS_HREDRAW | CS_DBLCLKS | CS_OWNDC;
        cls.lpfnWndProc    = (WNDPROC)AppWndProc;
        cls.cbWndExtra     = 0;
        cls.cbClsExtra     = 0;

        if (!RegisterClass(&cls))
            return FALSE;
    }

    DWORD Style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
    RECT WindowRect = { 0, 0 };
    WindowRect.right = WindowWidth;
    WindowRect.bottom = WindowHeight;

    AdjustWindowRect(&WindowRect,Style,TRUE);
    
    hwndApp = CreateWindow(szAppName,szAppName,Style,
                    CW_USEDEFAULT,0,
                    WindowRect.right-WindowRect.left,
                    WindowRect.bottom-WindowRect.top,
                    0,0,hInst,0);

    // create RC

    WindowDC = GetDC(hwndApp);
    PIXELFORMATDESCRIPTOR FormatDescriptor =
    {
        sizeof (PIXELFORMATDESCRIPTOR), 1,
        PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
        PFD_TYPE_RGBA, 16,
        0,0, 0,0, 0,0, 0,0,
        0, 0,0,0,0,
        16,
        0, 0,
        PFD_MAIN_PLANE,
        0,
        0,0,0
    };

#if 0
    int const NumberOfFormats = DescribePixelFormat(WindowDC,1,
           sizeof(FormatDescriptor),&FormatDescriptor);
    for(int i=0;i < NumberOfFormats;i++)
      { 
    DescribePixelFormat(WindowDC,i,
           sizeof(FormatDescriptor),&FormatDescriptor);
      }
#endif
    assert(!(GetDeviceCaps(WindowDC,RASTERCAPS) & RC_PALETTE));

    int FormatIndex = ChoosePixelFormat(WindowDC,&FormatDescriptor);
    SetPixelFormat(WindowDC,FormatIndex,&FormatDescriptor);
    WindowRC = wglCreateContext(WindowDC);
    wglMakeCurrent(WindowDC,WindowRC);

    // everything's set up, so call the app
    Initialize();
    
    ShowWindow(hwndApp,sw);

    return TRUE;
}


void AppExit()
{
    Destroy();

    wglMakeCurrent(0,0);
    wglDeleteContext(WindowRC);
    ReleaseDC(hwndApp,WindowDC);
}

BOOL AppIdle()
{
    if (fAppActive)
    {
        HDC WindowDC = GetDC(hwndApp);
        AppPaint(hwndApp,WindowDC);
        ReleaseDC(hwndApp,WindowDC);

        return FALSE;
    }
    else
    {
        return TRUE;        // nothing to do.
    }
}

int PASCAL WinMain(HINSTANCE hInst, HINSTANCE hPrev, LPSTR szCmdLine, int sw)
{
    MSG     msg;

    /* Call initialization procedure */
    if (!AppInit(hInst,hPrev,sw,szCmdLine))
    return FALSE;

    /*
     * Polling messages from event queue
     */
    for (;;)
    {
        if (PeekMessage(&msg, NULL, 0, 0,PM_REMOVE))
        {
            if (msg.message == WM_QUIT)
                break;

            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        else
        {
            if (AppIdle())
                WaitMessage();
        }
    }


    AppExit();
    return msg.wParam;
}

int AppPaint (HWND hwnd, HDC hdc)
{
    Execute(Zoom, height, viewAngle );      

    height += dheight;
    viewAngle += dviewAngle;
    Zoom += dZoom;

    SwapBuffers(WindowDC);

    return TRUE;
}

LONG FAR PASCAL AppWndProc(HWND hwnd,UINT msg,WPARAM wParam,LPARAM lParam)
{
    PAINTSTRUCT ps;
    HDC hdc;

    switch (msg)
    {
        case WM_CREATE:
        break;

        case WM_ACTIVATEAPP:
            fAppActive = (BOOL)wParam;
            break;
        case WM_KEYDOWN:
        {
            switch(wParam)
            {
                case VK_LEFT:
                {
                    dviewAngle = 0.3f;
                    break;
                }
                case VK_RIGHT:
                {
                    dviewAngle = -.3f;
                    break;
                }
                case VK_UP:
                {
                    dheight = .3f;
                    break;
                }
                case VK_DOWN:
                {
                    dheight = -.3f;
                    break;
                }
                case 'Z':
                {
                    dZoom = 1.0f;
                    break;
                }
                case 'X':
                {
                    dZoom = -1.0f;
                    break;
                }
                case 'G':
                {
                    SendMessage(hwnd,WM_COMMAND,UI_GRAVITY,0);
                    break;
                }
                case 'F':
                {
                    SendMessage(hwnd,WM_COMMAND,UI_FRICTION,0);
                    break;
                }

                case ' ':
                {
					Reset();					
                    break;
                }
            }
            break;
         }
        case WM_KEYUP:
        {
            switch(wParam)
            {
                case 'Z':
                case 'X':
                {
                    dZoom = 0.0f;
                    break;
                }
                case VK_LEFT:
                case VK_RIGHT:
                {
                    dviewAngle = 0.0f;
                    break;
                }
                case VK_UP:
                case VK_DOWN:
                {
                    dheight = 0.0f;
                    break;
                }
            }
            break;
        }

        case WM_LBUTTONDOWN:
        {
			int xPos = GET_X_LPARAM(lParam); 
			int yPos = GET_Y_LPARAM(lParam);
			
			selectObjectAtPoint(xPos,yPos);

            break;
        }

        case WM_MOUSEMOVE:
        {
            break;
        }

        case WM_LBUTTONUP:
        {
            break;
        }

        case WM_ERASEBKGND:
            break;

        case WM_INITMENU:
            break;

        case WM_COMMAND:
            return AppCommand(hwnd,msg,wParam,lParam);

        case WM_DESTROY:
            PostQuitMessage(0);
            break;

        case WM_CLOSE:
            break;

        case WM_PAINT:
            hdc = BeginPaint(hwnd,&ps);
            AppPaint (hwnd,hdc);
            EndPaint(hwnd,&ps);
            return 0L;

        case WM_SIZE:
            break;
    }
    return DefWindowProc(hwnd,msg,wParam,lParam);
}

LONG AppCommand (HWND hwnd,UINT ,WPARAM wParam,LPARAM )
{
    switch(wParam)
    {
        case UI_ABOUT:
            DialogBox(hInstApp,"AppAbout",hwnd,AppAbout);
            break;

        case UI_GRAVITY:
            ToggleGravity();
            break;

        case UI_FRICTION:
            ToggleFriction();
            break;

        case UI_EXIT:
            PostMessage(hwnd,WM_CLOSE,0,0L);
            break;
    }
    return 0L;
}
    