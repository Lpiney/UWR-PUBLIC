from pathlib import Path
from html import escape
from PIL import Image, ImageDraw, ImageFont
OUT=Path(__file__).parent
W,H=1080,820
S=2
im=Image.new('RGB',(W*S,H*S),'white');d=ImageDraw.Draw(im)
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">','<rect width="100%" height="100%" fill="white"/>']
P='#8B65D9';T='#232830';F='#F0EBFC';G='#55616F'
def font(n,b=False):return ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial'+(' Bold' if b else '')+'.ttf',n*S)
def text(x,y,s,n=18,b=False,center=False,color=T):
 a='middle' if center else 'start'
 svg.append(f'<text x="{x}" y="{y}" text-anchor="{a}" font-family="Arial,sans-serif" font-size="{n}" font-weight="{700 if b else 400}" fill="{color}">{escape(s)}</text>')
 d.text((x*S,y*S),s,font=font(n,b),fill=color,anchor='ms' if center else 'ls')
def line(x,y,xx,yy,col=T,dash=False,width=1.5):
 svg.append(f'<line x1="{x}" y1="{y}" x2="{xx}" y2="{yy}" stroke="{col}" stroke-width="{width}"'+(' stroke-dasharray="5 5"' if dash else '')+'/>')
 if dash:
  import math
  length=math.hypot(xx-x,yy-y)
  for t in range(0,int(length),10):
   a=t/length;bb=min(t+5,length)/length
   d.line((int((x+(xx-x)*a)*S),int((y+(yy-y)*a)*S),int((x+(xx-x)*bb)*S),int((y+(yy-y)*bb)*S)),fill=col,width=int(width*S))
 else:d.line((x*S,y*S,xx*S,yy*S),fill=col,width=int(width*S))
def rect(x,y,w,h,fill=None,dash=False):
 svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="3" fill="{fill or "none"}" stroke="{P}"'+(' stroke-dasharray="4 4"' if dash else '')+'/>')
 if fill:d.rectangle((x*S,y*S,(x+w)*S,(y+h)*S),fill=fill)
 for coords in [(x,y,x+w,y),(x+w,y,x+w,y+h),(x,y+h,x+w,y+h),(x,y,x,y+h)]:line(*coords,P,dash)
X=[110,325,540,755,970]
def msg(a,b,y,s,reply=False):
 x,xx=X[a],X[b];line(x,y,xx,y,dash=reply)
 sign=1 if xx>x else -1
 points=[(xx,y),(xx-sign*9,y-4),(xx-sign*9,y+4)]
 svg.append('<polygon points="'+' '.join(f'{a},{b}' for a,b in points)+f'" fill="{T}"/>')
 d.polygon([(a*S,b*S) for a,b in points],fill=T)
 text((x+xx)/2,y-10,s,17,center=True)
def selfmsg(a,y,s):
 x=X[a];line(x,y,x+55,y);line(x+55,y,x+55,y+25);line(x+55,y+25,x,y+25)
 line(x,y+25,x+9,y+20);line(x,y+25,x+9,y+30)
 text(x+65,y+17,s,17)
def panel(y,title,names,end):
 text(40,y,title,24,True)
 for x,name in zip(X,names):
  rect(x-110,y+25,220,48,F);text(x,y+56,name,18,center=True)
  line(x,y+73,x,end,P)
def frame(y,h,label,sub=''):
 rect(65,y,1160,h,dash=True);rect(65,y,75,26,F);text(77,y+19,label,16,True)
 if sub:text(650,y+21,sub,16,center=True)
text(30,40,'U.P. | Coding Flow',28,True)
for x,name in zip(X,['Operator','Laptop','ESP32','ESCs / thrusters','HTTP tower']):
 rect(x-90,64,180,44,F);text(x,92,name,18,center=True)
 line(x,108,x,775,P)
def band(y,s):
 rect(25,y,1030,29,F);text(40,y+21,s,18,True)
band(130,'1. Driving and safety')
msg(0,1,195,'Control input')
msg(1,2,242,'Send commands')
rect(440,263,420,93,dash=True)
text(455,284,'Safety check',17,True)
msg(2,3,313,'Drive if enabled')
msg(2,3,346,'Otherwise: neutral')
msg(2,1,392,'Return status',True)
band(421,'2. Vision assistance')
msg(0,1,480,'Capture target')
selfmsg(1,511,'Recognise tag / colour')
msg(1,0,575,'Show result; save if recognised',True)
band(608,'3. Wi-Fi mission')
msg(2,4,664,'HTTP GET')
msg(4,2,710,'Mission data',True)
msg(2,1,757,'Display data',True)
text(30,805,'Manual piloting continues during vision and network tasks.',17,color=G)
svg.append('</svg>')
(OUT/'UP_Coding_Sequence.svg').write_text('\n'.join(svg))
im.save(OUT/'UP_Coding_Sequence.png')
print(OUT/'UP_Coding_Sequence.png')
