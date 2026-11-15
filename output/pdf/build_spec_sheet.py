from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.colors import HexColor, white
from reportlab.platypus import Paragraph
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.utils import ImageReader
from pathlib import Path

OUT=Path(__file__).parent
W,H=A4
INK=HexColor('#172D3B'); BLUE=HexColor('#155E75'); LIGHT=HexColor('#EDF4F6'); GREY=HexColor('#52616B'); AMBER=HexColor('#9A5B12')
c=canvas.Canvas(str(OUT/'Underwater_Potatoes_UP_Spec_Sheet_DRAFT.pdf'),pagesize=A4)
c.setTitle('Underwater Potatoes | U.P. Technical Spec Sheet | Draft')
c.setAuthor('Underwater Potatoes')
style=ParagraphStyle('body',fontName='Helvetica',fontSize=12,leading=15,textColor=INK)
def txt(x,t,s,size=12,bold=False,color=INK):
 c.setFillColor(color);c.setFont('Helvetica-Bold' if bold else 'Helvetica',size);c.drawString(x,H-t-size,s)
def para(x,t,w,s,size=12,leading=15):
 st=ParagraphStyle('p',parent=style,fontSize=size,leading=leading)
 p=Paragraph(s,st);pw,ph=p.wrap(w,1000);p.drawOn(c,x,H-t-ph);return ph

def section(t,s):
 c.setStrokeColor(BLUE);c.setLineWidth(1);c.line(34,H-t-20,W-34,H-t-20)
 txt(34,t,s,13,True,BLUE)
def box(x,t,w,h,lines,dashed=False):
 c.setFillColor(LIGHT if not dashed else white);c.setStrokeColor(AMBER if dashed else BLUE);c.setLineWidth(.9)
 c.setDash(4,3) if dashed else c.setDash()
 c.roundRect(x,H-t-h,w,h,5,fill=1,stroke=1);c.setDash()
 if isinstance(lines,str):lines=[lines]
 start=t+(h-len(lines)*14)/2
 for i,s in enumerate(lines):
  c.setFillColor(INK);c.setFont('Helvetica',12);c.drawCentredString(x+w/2,H-start-i*14-11,s)
def line(x1,t1,x2,t2,dashed=False,arrow=False):
 c.setStrokeColor(AMBER if dashed else BLUE);c.setLineWidth(1);c.setDash(4,3) if dashed else c.setDash()
 c.line(x1,H-t1,x2,H-t2);c.setDash()
 if arrow:
  import math
  a=math.atan2(t2-t1,x2-x1)
  for da in [-.5,.5]:
   c.line(x2,H-t2,x2-6*math.cos(a+da),H-(t2-6*math.sin(a+da)))
def footer(n):
 txt(34,805,'DRAFT | 22 October 2026 | [TBC] = to be confirmed',12,color=GREY)
 txt(W-64,805,f'{n}/2',12,color=GREY)
def header(sub):
 txt(34,27,'UNDERWATER POTATOES',21,True)
 txt(34,56,sub,12,color=BLUE)

header('U.P. | CityUHK UR Fall Training 2026 | Technical Spec Sheet')
section(86,'TEAM & VEHICLE')
c.drawImage(str(OUT/'team_photo.jpg'),34,H-118-172,width=230,height=172,preserveAspectRatio=True,anchor='c',mask='auto')
para(280,116,281,'<b>ROV:</b> U.P.<br/><b>Dimensions:</b> 0.22 x 0.25 x 0.43 m<br/>[TBC: length / width / height order]<br/><b>Design &amp; build:</b> 50 person-hours<br/>[TBC: team cumulative total]<br/><b>Propulsion:</b> 2 horizontal + 1 vertical<br/><b>Controller:</b> ESP32-S3<br/><b>Supply:</b> [TBC: voltage / source]')
para(34,302,527,'<b>Team photo:</b> [TBC: all members present; map names to photo positions].<br/><b>Roles:</b> Kingston - Project lead; Jeff - CAD; Bruce - Programming;<br/>John - Code review; Brian - Circuits; Kelson - Documentation; Isaac - PR.')
section(371,'SAFETY FEATURES')
para(34,402,527,'<b>Physical protection.</b> Waterproof enclosure limits water ingress; thruster guards reduce contact with rotating blades. [TBC: final installation].<br/><b>Safety lock &amp; link protection.</b> Locked by default; Y toggles permission to drive. Controller loss re-locks. No valid command for 200 ms (default) requests neutral; outputs then ramp down.<br/><b>Controlled output.</b> A 3 s startup neutral window, gradual throttle changes and a 520 ms neutral hold before reversal reduce abrupt commands. Default travel is capped at 30%; this is not a measured thrust or power limit.')
section(558,'SPECIAL FEATURES')
para(34,589,527,'<b>Integrated piloting.</b> Live video, link state, throttle and mission results share one display. Two horizontal thrusters provide surge/yaw; one provides heave.<br/><b>Vision assistance.</b> Operator-triggered AprilTag 36h11 identification supports maximum/minimum ID selection. Pointer-based HSV sampling identifies red, yellow and blue; successful results save annotated screenshots.<br/><b>Independent network task.</b> ESP32 performs Wi-Fi/HTTP retrieval on core 0; propulsion control runs on core 1. Mission movement remains manual.')
box(34,730,527,57,['VEHICLE PHOTO [TO INSERT]', 'Adjustable thruster mounts [TBC: mechanism and benefit]'],True)
footer(1);c.showPage()

header('SYSTEM ARCHITECTURE | Signal paths, control logic and circuit connections')
section(85,'1  ELECTRICAL FLOW CHART')
box(34,117,125,38,['Xbox controller']);box(215,117,137,38,['Laptop / display']);box(410,117,151,38,['ESP32-S3'])
line(159,136,215,136,arrow=True);txt(167,117,'USB')
line(352,136,410,136,arrow=True);txt(355,157,'Serial')
box(215,189,137,32,'Camera');line(283,189,283,155,arrow=True)
box(410,189,151,32,'3 ESCs / thrusters');line(486,155,486,189,arrow=True);txt(495,163,'PWM')
box(34,189,153,32,'Power path [TBC]',True)
para(34,230,527,'Camera interface / power distribution [TBC]. ESP32 also exchanges Wi-Fi/HTTP data with the mission tower. Power wiring is detailed below.')
section(269,'2  PROGRAM FLOW CHART')
txt(34,298,'LAPTOP (Python)',12,True);txt(314,298,'ESP32-S3 (firmware)',12,True)
box(34,320,247,30,'Read pad; update safety lock')
box(34,366,247,30,'Send axes + enable at 50 Hz')
box(34,412,247,44,['Preview / capture / recognise;', 'display results + save image'])
line(157,350,157,366,arrow=True);line(157,396,157,412,arrow=True)
box(314,320,247,30,'Parse serial; valid link + enabled?')
box(314,366,247,30,'Yes: mix axes | No: neutral')
box(314,412,247,44,['Trim + cap; startup / slew /', 'guard; update ~125 Hz / PWM 50 Hz'])
line(438,350,438,366,arrow=True);line(438,396,438,412,arrow=True)
line(281,381,314,335,arrow=True)
para(34,465,527,'Both loops repeat; firmware returns telemetry. Core 0 separately runs Wi-Fi/HTTP and queues results. Vision assists the operator; no autonomous navigation.')
section(506,'3  CIRCUIT DIAGRAM (CONTROL + POWER)')
# Actual connections encoded as a simplified schematic. Unverified supply is dashed.
box(34,540,151,125,[])
for t,s in [(548,'ESP32-S3'),(576,'GPIO4'),(590,'GPIO7'),(604,'GPIO8'),(618,'GND')]:
 txt(47,t,s)
for y,label in [(564,'LEFT'),(605,'RIGHT'),(646,'VERTICAL')]:
 box(302,y-14,114,29,f'{label} ESC')
 box(467,y-14,94,29,'Thruster')
 line(416,y,467,y,arrow=True)
 # route signal from board labels to each corresponding ESC
for sy,ey in [(582,564),(596,605),(610,646)]:
 line(185,sy,302,ey)
txt(201,542,'50 Hz PWM')
# Common signal ground, with branches to ESC signal ground
line(185,624,245,624);line(245,574,245,675)
for y in [574,615,656]:
 line(245,y,302,y)
 c.setFillColor(BLUE);c.circle(245,H-y,2,fill=1,stroke=0)
txt(34,680,'Common GND: ESP32 + all ESC signal grounds',12)
box(34,707,197,42,['DC supply [TBC]', 'Voltage / fuse / switch'],True)
line(132,749,132,754,True);line(132,754,437,754,True);line(437,574,437,754,True)
for y in [574,615,656]:line(437,y,416,y,True)
para(249,697,182,'Dashed: DC path [TBC].<br/>USB board supply [TBC].<br/>ESC BEC red: isolated.')
para(34,770,527,'Confirm ratings, connectors and final wiring before use. Defaults may be overridden by stored settings; vehicle test results [TBC].')
footer(2);c.save()
print(OUT/'Underwater_Potatoes_UP_Spec_Sheet_DRAFT.pdf')
