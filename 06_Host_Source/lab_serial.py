"""Native Windows COM 8N1, bounded writes, no intentional DTR/RTS pulse."""
import ctypes
from ctypes import wintypes
import re
import sys

class DCB(ctypes.Structure):
    _fields_=[('DCBlength',wintypes.DWORD),('BaudRate',wintypes.DWORD),('flags',wintypes.DWORD),
              ('wReserved',wintypes.WORD),('XonLim',wintypes.WORD),('XoffLim',wintypes.WORD),
              ('ByteSize',wintypes.BYTE),('Parity',wintypes.BYTE),('StopBits',wintypes.BYTE),
              ('XonChar',ctypes.c_char),('XoffChar',ctypes.c_char),('ErrorChar',ctypes.c_char),
              ('EofChar',ctypes.c_char),('EvtChar',ctypes.c_char),('wReserved1',wintypes.WORD)]
class Timeouts(ctypes.Structure):
    _fields_=[(name,wintypes.DWORD) for name in ('interval','read_multiplier','read_constant','write_multiplier','write_constant')]

class Serial:
    def __init__(self,port,baud=115200):
        if sys.platform!='win32':raise OSError('实串口模式需要Windows')
        if not re.fullmatch(r'COM[1-9][0-9]*',port.upper()):raise ValueError('串口格式应为COM3等')
        self.handle=None;self.api=ctypes.WinDLL('kernel32',use_last_error=True)
        self.api.CreateFileW.argtypes=[wintypes.LPCWSTR,wintypes.DWORD,wintypes.DWORD,ctypes.c_void_p,wintypes.DWORD,wintypes.DWORD,wintypes.HANDLE]
        self.api.CreateFileW.restype=wintypes.HANDLE
        for name,args in (
            ('GetCommState',[wintypes.HANDLE,ctypes.POINTER(DCB)]),
            ('SetCommState',[wintypes.HANDLE,ctypes.POINTER(DCB)]),
            ('SetCommTimeouts',[wintypes.HANDLE,ctypes.POINTER(Timeouts)]),
            ('ReadFile',[wintypes.HANDLE,ctypes.c_void_p,wintypes.DWORD,ctypes.POINTER(wintypes.DWORD),ctypes.c_void_p]),
            ('WriteFile',[wintypes.HANDLE,ctypes.c_void_p,wintypes.DWORD,ctypes.POINTER(wintypes.DWORD),ctypes.c_void_p]),
            ('CloseHandle',[wintypes.HANDLE])):
            function=getattr(self.api,name);function.argtypes=args;function.restype=wintypes.BOOL
        self.handle=self.api.CreateFileW('\\\\.\\'+port.upper(),0xC0000000,0,None,3,0,None)
        if self.handle==ctypes.c_void_p(-1).value:
            self.handle=None;raise ctypes.WinError(ctypes.get_last_error())
        try:
            dcb=DCB();dcb.DCBlength=ctypes.sizeof(DCB)
            self.check(self.api.GetCommState(self.handle,ctypes.byref(dcb)))
            dcb.BaudRate=baud;dcb.ByteSize=8;dcb.Parity=0;dcb.StopBits=0
            dcb.flags=(dcb.flags & ((3<<4)|(3<<12)))|1
            self.check(self.api.SetCommState(self.handle,ctypes.byref(dcb)))
            self.check(self.api.SetCommTimeouts(self.handle,ctypes.byref(Timeouts(0xFFFFFFFF,0,0,0,100))))
        except Exception:self.close();raise
    @staticmethod
    def check(ok):
        if not ok:raise ctypes.WinError(ctypes.get_last_error())
    def read(self):
        buf=ctypes.create_string_buffer(8192);n=wintypes.DWORD()
        self.check(self.api.ReadFile(self.handle,buf,len(buf),ctypes.byref(n),None));return buf.raw[:n.value]
    def write(self,data):
        buf=ctypes.create_string_buffer(data);n=wintypes.DWORD()
        self.check(self.api.WriteFile(self.handle,buf,len(data),ctypes.byref(n),None))
        if n.value!=len(data):raise OSError('串口仅发送部分数据，参数状态未知')
    def close(self):
        if self.handle is not None:self.api.CloseHandle(self.handle);self.handle=None

def ports():
    import winreg
    found=set()
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,r'HARDWARE\DEVICEMAP\SERIALCOMM') as key:
            for i in range(winreg.QueryInfoKey(key)[1]):found.add(str(winreg.EnumValue(key,i)[1]))
    except OSError:pass
    return sorted(found,key=lambda port:int(re.sub(r'\D','',port) or 0))
