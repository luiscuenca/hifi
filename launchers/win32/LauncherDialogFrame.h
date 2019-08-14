#pragma once


// CLauncherDialogFrame dialog

class CLauncherDialogFrame : public CDialog
{
	DECLARE_DYNAMIC(CLauncherDialogFrame)

public:
	CLauncherDialogFrame(CWnd* pParent = nullptr);   // standard constructor
	virtual ~CLauncherDialogFrame();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CLauncherDialogFrame };
#endif

protected:
    bool _shouldLaunch { TRUE };
    HWND _interfaceWindow { nullptr };

    virtual BOOL OnInitDialog();
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    afx_msg void OnTimer(UINT_PTR nIDEvent);
	DECLARE_MESSAGE_MAP()
};
