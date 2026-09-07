# -*- coding: utf-8 -*-
"""16-sensor ring diagram, neon-on-black to match the team's poster style."""
import math, io

NEON = '#2BEA7C'
DIM = '#15past'
DIM = '#157F47'
YELLOW = '#F2C94C'
TEXT = '#EAF6EE'
MUTE = '#7FA890'
BODY = '#0A1710'

o = io.StringIO(); w = o.write
W = H = 600; cx = cy = 300
Rs = 196; Rn = 248; Rbody = 158

w('<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg" style="width:100%%;height:auto;display:block">\n' % (W, H))
w('<circle cx="%d" cy="%d" r="%d" fill="%s" stroke="%s" stroke-width="2.5"/>\n' % (cx, cy, Rbody, BODY, NEON))
w('<circle cx="%d" cy="%d" r="%d" fill="none" stroke="%s" stroke-width="1.5" stroke-dasharray="5 8"/>\n' % (cx, cy, Rs + 22, DIM))

for i in range(16):
    a = math.radians(-90 + i * 22.5)
    x1 = cx + math.cos(a) * (Rbody - 6); y1 = cy + math.sin(a) * (Rbody - 6)
    x2 = cx + math.cos(a) * (Rs - 14); y2 = cy + math.sin(a) * (Rs - 14)
    w('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="1.2"/>\n' % (x1, y1, x2, y2, DIM))

HL = {0: YELLOW, 3: NEON}
for i in range(16):
    ang = -90 + i * 22.5; ar = math.radians(ang)
    x = cx + math.cos(ar) * Rs; y = cy + math.sin(ar) * Rs
    col = HL.get(i, DIM)
    fill = col if i in HL else 'none'
    w('<g transform="translate(%.1f %.1f) rotate(%.1f)">' % (x, y, ang + 90))
    w('<rect x="-13" y="-10" width="26" height="20" rx="4" fill="%s" stroke="%s" stroke-width="2.5"/>' % (fill, col))
    w('</g>\n')
    nx = cx + math.cos(ar) * Rn; ny = cy + math.sin(ar) * Rn
    ncol = col if i in HL else MUTE
    w('<text x="%.1f" y="%.1f" fill="%s" font-size="22" font-weight="%s" '
      'font-family="Space Grotesk, ui-monospace, monospace" text-anchor="middle" '
      'dominant-baseline="central">%d</text>\n' % (nx, ny, ncol, '700' if i in HL else '500', i))

Ra = Rbody + 22
x0 = cx + math.cos(math.radians(-90)) * Ra; y0 = cy + math.sin(math.radians(-90)) * Ra
x1 = cx + math.cos(math.radians(-67.5)) * Ra; y1 = cy + math.sin(math.radians(-67.5)) * Ra
w('<path d="M %.1f %.1f A %d %d 0 0 1 %.1f %.1f" fill="none" stroke="%s" stroke-width="3"/>\n'
  % (x0, y0, Ra, Ra, x1, y1, YELLOW))
am = math.radians(-78.75)
w('<text x="%.1f" y="%.1f" fill="%s" font-size="24" font-weight="700" '
  'font-family="Space Grotesk, ui-monospace, monospace" text-anchor="middle" '
  'dominant-baseline="central">22.5&#176;</text>\n'
  % (cx + math.cos(am) * (Ra + 36), cy + math.sin(am) * (Ra + 36), YELLOW))

am5 = math.radians(-90 + 5 * 22.5)
w('<defs><marker id="rh" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" '
  'orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="%s"/></marker></defs>\n' % NEON)
w('<line x1="%d" y1="%d" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="5" marker-end="url(#rh)"/>\n'
  % (cx, cy, cx + math.cos(am5) * (Rbody - 20), cy + math.sin(am5) * (Rbody - 20), NEON))
w('<circle cx="%d" cy="%d" r="7" fill="%s"/>\n' % (cx, cy, NEON))

w('<text x="%d" y="%d" fill="%s" font-size="25" font-weight="700" '
  'font-family="Vazirmatn, Tahoma, sans-serif" text-anchor="middle">'
  '&#1580;&#1607;&#1578; &#1581;&#1585;&#1705;&#1578;</text>\n' % (cx, cy + 58, TEXT))
w('<text x="%d" y="%d" fill="%s" font-size="24" font-weight="700" '
  'font-family="Space Grotesk, ui-monospace, monospace" text-anchor="middle" '
  'direction="ltr">move(3 + 2)</text>\n' % (cx, cy + 92, NEON))
w('<text x="%d" y="%d" fill="%s" font-size="23" font-weight="700" '
  'font-family="Space Grotesk, ui-monospace, monospace" text-anchor="middle" '
  'direction="ltr">FRONT &#183; 0</text>\n' % (cx, cy - 86, YELLOW))
w('</svg>')

open('ring.svg', 'w', encoding='utf8').write(o.getvalue())
print('ring.svg', len(o.getvalue()), 'bytes')
