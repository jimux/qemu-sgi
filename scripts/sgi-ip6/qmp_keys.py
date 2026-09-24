import socket,json,time,sys
import os
SOCK=os.environ.get("QMP_SOCK","/tmp/opencode/gfx_qmp.sock")
SHIFT={'(':('9',True),')':('0',True),'_':('minus',True),':':('semicolon',True),
       '!':('1',True),'@':('2',True),'#':('3',True)}
Q={' ':'spc','-':'minus','=':'equal','/':'slash','.':'dot',',':'comma',';':'semicolon',
   "'":'apostrophe','[':'bracket_left',']':'bracket_right','\\':'backslash','`':'grave_accent',
   '\n':'ret','\t':'tab'}
def kc(c):
    if c in SHIFT: return SHIFT[c]
    if c in Q: return (Q[c],False)
    if c.isalpha(): return (c.lower(), c.isupper())
    if c.isdigit(): return (c,False)
    raise ValueError(c)
class Qmp:
    def __init__(s):
        s.s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM); s.s.connect(SOCK); s.f=s.s.makefile("rwb")
        s.cmd({"execute":"qmp_capabilities"})
    def cmd(s,o):
        s.f.write((json.dumps(o)+"\n").encode()); s.f.flush()
        while True:
            l=s.f.readline()
            if not l: return None
            l=l.strip()
            if l.startswith(b"{"):
                try:
                    o=json.loads(l)
                    if "event" in o: continue
                    return o
                except: continue
    def key(s,q,down):
        return s.cmd({"execute":"input-send-event","arguments":{"events":[{"type":"key","data":{"down":down,"key":{"type":"qcode","data":q}}}]}})
    def press(s,c):
        q,sh=kc(c)
        if sh: s.key("shift",True)
        s.key(q,True); s.key(q,False)
        if sh: s.key("shift",False)
        time.sleep(0.05)
    def typ(s,text):
        for c in text: s.press(c)
    def dump(s,p):
        return s.cmd({"execute":"screendump","arguments":{"filename":p}})
if __name__=="__main__":
    q=Qmp()
    cmd=sys.argv[1]
    if cmd=="typ": q.typ(sys.argv[2])
    elif cmd=="dump": time.sleep(0.5); q.dump(sys.argv[2]); time.sleep(1.5)
    print("ok")
