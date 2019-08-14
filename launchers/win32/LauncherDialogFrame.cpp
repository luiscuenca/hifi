// CLauncherDialogFrame.cpp : implementation file
//

#include "stdafx.h"
#include "LauncherApp.h"
#include "LauncherDialogFrame.h"
#include "resource.h"

// CLauncherDialogFrame dialog

IMPLEMENT_DYNAMIC(CLauncherDialogFrame, CDialog)

CLauncherDialogFrame::CLauncherDialogFrame(CWnd* pParent)
	: CDialog(IDD_DIALOG1, pParent)
{
    AfxGetApp()->LoadIcon(IDR_MAINFRAME);
    _shouldLaunch = true;
}

CLauncherDialogFrame::~CLauncherDialogFrame()
{
}

void CLauncherDialogFrame::OnTimer(UINT_PTR nIDEvent) {
    if (_shouldLaunch) {
        PROCESS_INFORMATION processInformation = { 0 };
        STARTUPINFO startupInfo = { 0 };
        startupInfo.cb = sizeof(startupInfo);

        CString installDir;
        theApp._manager.getAndCreatePaths(LauncherManager::PathType::Interface_Directory, installDir);
        CString interfaceExe = installDir + _T("\\interface.exe");
        int nStrBuffer = interfaceExe.GetLength() + 50;


        // Create the process
        BOOL result = CreateProcess(interfaceExe.GetBuffer(nStrBuffer), interfaceExe.GetBuffer(nStrBuffer),
            NULL, NULL, FALSE,
            NORMAL_PRIORITY_CLASS,
            NULL, NULL, &startupInfo, &processInformation);
        interfaceExe.ReleaseBuffer();


        // Start the child process. 
        if (!result)
        {
            TRACE("CreateProcess failed (%d).\n", GetLastError());
            return;
        }

        // Wait until child process exits.
        WaitForSingleObject(processInformation.hProcess, INFINITE);

        // Close process and thread handles. 
        CloseHandle(processInformation.hProcess);
        CloseHandle(processInformation.hThread);
        _shouldLaunch = false;
    } 
}
BOOL CLauncherDialogFrame::OnInitDialog() {
    CDialog::OnInitDialog();
    SetTimer(1, 2, NULL);
    return TRUE;
}
void CLauncherDialogFrame::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CLauncherDialogFrame, CDialog)
    ON_WM_TIMER()
END_MESSAGE_MAP()


// CLauncherDialogFrame message handlers
