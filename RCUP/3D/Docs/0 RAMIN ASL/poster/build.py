# -*- coding: utf-8 -*-
"""Builds Main.dc.html — SaraTech Junior Soccer Lightweight Primary poster.

Follows the team's existing secondary poster: black ground, neon-green frames,
two spec columns, the robot cut out at the bottom. Primary changes: no shoot
system, no out-line algorithm, TDAxis13 in place of GY-25, primary robot photo.
"""
import json

ring = open('ring.svg', encoding='utf8').read()

NEON = '#2BEA7C'
NEON_LINE = '#17A85C'
YELLOW = '#F2C94C'
TEXT = '#EAF6EE'
MUTE = '#8FB3A0'
BG = '#040A06'

GLOW = 'text-shadow: 0 0 18px rgba(43,234,124,0.55);'
PANEL = (
    'border: 2px solid ' + NEON_LINE + '; border-radius: 22px; '
    'background: linear-gradient(180deg, rgba(10,26,17,0.88), rgba(4,12,8,0.88)); '
    'box-shadow: 0 0 30px rgba(43,234,124,0.16), inset 0 0 70px rgba(43,234,124,0.045); '
    'padding: 32px 30px 36px 30px; display: flex; flex-direction: column; gap: 26px;'
)
FRAME = (
    'width: 218px; height: 158px; flex-shrink: 0; border: 2px solid ' + NEON_LINE + '; '
    'border-radius: 14px; background: #F4F7F5; box-shadow: 0 0 16px rgba(43,234,124,0.20); '
    'display: flex; align-items: center; justify-content: center; overflow: hidden; padding: 8px;'
)
IMG = 'max-width: 100%; max-height: 100%; width: auto; height: auto; display: block; object-fit: contain;'


def hexbadge(glyph):
    """Hexagon outline with a small stroke glyph inside — the section marker."""
    return (
        '<svg viewBox="0 0 100 100" style="width: 74px; height: 74px; flex-shrink: 0; display: block;" '
        'xmlns="http://www.w3.org/2000/svg">'
        '<path d="M50 4 L91 27 L91 73 L50 96 L9 73 L9 27 Z" fill="none" stroke="' + NEON + '" stroke-width="3"/>'
        + glyph +
        '</svg>'
    )


GEAR = (
    '<g transform="translate(50 50)" fill="none" stroke="' + NEON + '" stroke-width="3.4" '
    'stroke-linecap="round"><circle r="11"/><circle r="21"/>'
    '<path d="M0 -21 L0 -29 M0 21 L0 29 M-21 0 L-29 0 M21 0 L29 0 '
    'M-15 -15 L-20.5 -20.5 M15 15 L20.5 20.5 M15 -15 L20.5 -20.5 M-15 15 L-20.5 20.5"/></g>'
)
CHIP = (
    '<g fill="none" stroke="' + NEON + '" stroke-width="3.4" stroke-linecap="round">'
    '<rect x="35" y="35" width="30" height="30" rx="4"/>'
    '<path d="M43 35 L43 25 M57 35 L57 25 M43 65 L43 75 M57 65 L57 75 '
    'M35 43 L25 43 M35 57 L25 57 M65 43 L75 43 M65 57 L75 57"/></g>'
)
FLOW = (
    '<g fill="none" stroke="' + NEON + '" stroke-width="3.4" stroke-linecap="round" stroke-linejoin="round">'
    '<circle cx="50" cy="50" r="9"/><circle cx="50" cy="50" r="24" stroke-dasharray="6 9"/>'
    '<path d="M50 26 L50 18 M62 71 L67 78"/></g>'
)


def bullets(items):
    rows = []
    for t in items:
        rows.append(
            '<div style="display: flex; gap: 14px; align-items: flex-start;">'
            '<div style="width: 11px; height: 11px; border-radius: 50%; background: ' + NEON + '; '
            'box-shadow: 0 0 10px rgba(43,234,124,0.8); margin-top: 13px; flex-shrink: 0;"></div>'
            '<div style="font-size: 25px; line-height: 1.62; color: ' + TEXT + ';">' + t + '</div>'
            '</div>'
        )
    return '<div style="display: flex; flex-direction: column; gap: 10px;">' + ''.join(rows) + '</div>'


def block(title, items, photo, alt, note=None):
    if photo:
        media = ('<div style="' + FRAME + '"><img src="' + photo + '" alt="' + alt + '" style="' + IMG + '"></div>')
    else:
        media = (
            '<div style="' + FRAME.replace('background: #F4F7F5;', 'background: #08130C;') + '">'
            '<div style="text-align: center; padding: 0 10px;">'
            '<div style="font-family: \'Space Grotesk\', monospace; font-size: 24px; font-weight: 700; '
            'color: ' + NEON + ';">' + alt + '</div>'
            '<div style="font-size: 18px; color: ' + MUTE + '; margin-top: 8px;">' + note + '</div>'
            '</div></div>'
        )
    return (
        '<div style="display: flex; gap: 24px; align-items: flex-start;">'
        '<div style="flex: 1; display: flex; flex-direction: column; gap: 12px;">'
        '<div style="display: flex; align-items: center; gap: 16px;">'
        '<div style="font-size: 32px; font-weight: 800; color: ' + NEON + '; ' + GLOW + '">' + title + '</div>'
        '<div style="flex: 1; height: 0; border-top: 2px dashed ' + NEON_LINE + ';"></div>'
        '</div>'
        + bullets(items) +
        '</div>' + media + '</div>'
    )


def panel_header(title, glyph):
    return (
        '<div style="display: flex; align-items: center; gap: 20px;">'
        + hexbadge(glyph) +
        '<div style="font-size: 48px; font-weight: 900; color: ' + NEON + '; ' + GLOW + '">' + title + '</div>'
        '<div style="flex: 1; height: 3px; background: linear-gradient(to left, ' + NEON_LINE + ', transparent);"></div>'
        '</div>'
    )


def en(t):
    return '<span class="en">' + t + '</span>'


# ---------------------------------------------------------------- mechanics --
mech = panel_header('مکانیک ربات', GEAR) + block(
    'شاسی ربات',
    ['طراحی بدنه ربات با ' + en('Solid Works') + ' و ' + en('Corel Draw'),
     'سه پلیت اصلی برای سنسورها، موتورها و ' + en('PCB'),
     'قطعات محافظ جانبی با پرینت سه‌بعدی'],
    'p-chassis.jpg', 'شاسی و پلیت موتورها',
) + block(
    'سیستم حرکتی ربات',
    ['استفاده از چرخ خورشیدی',
     'چهار موتور با زاویه تقریبی ۹۰ درجه',
     'آزادی عمل بالا و حرکت در همه جهات'],
    'p-wheel.jpg', 'چرخ خورشیدی',
) + block(
    'موتورها',
    ['حداکثر ولتاژ ۹ ولت',
     'سرعت ' + en('555 RPM'),
     'موتورهای ' + en('kgb') + ' ساخت تایوان'],
    'p-motor.jpg', 'موتور',
)

# -------------------------------------------------------------- electronics --
elec = panel_header('الکترونیک ربات', CHIP) + block(
    'طراحی مدار',
    ['استفاده از ' + en('Altium Designer'),
     'طراحی ' + en('PCB') + ' دولایه'],
    'p-pcb.jpg', 'طراحی برد',
) + block(
    'پردازنده ' + en('STM32'),
    ['استفاده از ماژول ' + en('STM32') + ' با پردازنده ' + en('ARM'),
     'پردازش سریع داده‌ها',
     'مصرف بهینه و کنترل مرکزی ربات'],
    'p-stm32.jpg', 'ماژول STM32',
) + block(
    'درایور ' + en('L6203'),
    ['کنترل سرعت و جهت موتور',
     'ولتاژ تغذیه ۴۲ ولت و جریان ۵ آمپر',
     'عملکرد پایدار و دقیق'],
    'p-driver.jpg', 'درایور L6203',
) + block(
    'سنسورهای تشخیص توپ',
    ['گیرنده مادون قرمز ' + en('TSOP1140'),
     'حساس به امواج ۴۰ کیلوهرتز',
     'استفاده از ۱۶ سنسور با فاصله ۲۲/۵ درجه اطراف ربات'],
    'p-tsop.jpg', 'گیرنده مادون قرمز',
) + block(
    'سنسور زاویه‌سنج',
    ['ماژول ' + en('TDAxis13') + ' با دقت بالا',
     'تشخیص جهت، زاویه و شتاب',
     'حفظ زاویه ربات نسبت به دروازه حریف'],
    None, 'TDAxis13', note='تصویر ماژول',
) + block(
    'سنسور فاصله‌سنج',
    ['استفاده از ۳ سنسور ' + en('GP2Y0A02'),
     'سنسورهای ساخت شرکت ' + en('Sharp'),
     'تشخیص فاصله از دیواره‌ها و دروازه'],
    'p-sharp.jpg', 'فاصله‌سنج شارپ',
)

# ---------------------------------------------------------------- algorithm --
algo = (
    panel_header('الگوریتم تعقیب توپ', FLOW) +
    '<div style="display: flex; gap: 26px; align-items: center;">'
    '<div style="flex: 1; display: flex; flex-direction: column; gap: 14px;">'
    + bullets([
        'پویش ۱۶ سنسور و یافتن کوچک‌ترین مقدار ' + en('tsopMin'),
        'انتخاب حرکت با دو شماره اختلاف نسبت به سنسور فعال',
        'افزودن زاویه ' + en('TDAxis13') + ' به هر چهار موتور',
        'بازگشت به دروازه با اختلاف دو فاصله‌سنج کناری',
    ]) +
    '</div>'
    '<div style="width: 430px; flex-shrink: 0;">' + ring + '</div>'
    '</div>'
)

TEAM = ['محمدصدرا مظهر', 'سپنتا بشیری‌پور', 'امیررضا صادقی',
        'سینا غنی‌طوس', 'محمدپارسا فراتی', 'اشکان فرجی']

PERSON = (
    '<svg viewBox="0 0 24 24" style="width: 52px; height: 52px;" fill="none" stroke="' + NEON_LINE + '" '
    'stroke-width="1.6" stroke-linecap="round" xmlns="http://www.w3.org/2000/svg">'
    '<circle cx="12" cy="8.5" r="4"/><path d="M4.5 20a7.5 7.5 0 0 1 15 0"/></svg>'
)

team_html = ''.join(
    '<div style="display: flex; flex-direction: column; align-items: center; gap: 12px; width: 132px;">'
    '<div style="width: 112px; height: 112px; border-radius: 50%; border: 2px solid ' + NEON_LINE + '; '
    'background: #08130C; box-shadow: 0 0 16px rgba(43,234,124,0.25); display: flex; '
    'align-items: center; justify-content: center;">' + PERSON + '</div>'
    '<div style="font-size: 20px; color: ' + TEXT + '; text-align: center; line-height: 1.35;">' + n + '</div>'
    '</div>' for n in TEAM
)

HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <script src="./support.js"></script>
</head>
<body>
<x-dc>
<helmet>
  <link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Vazirmatn:wght@300;400;500;700;800;900&family=Space+Grotesk:wght@400;500;700&display=swap">
  <style>
    body { margin: 0; font-family: Vazirmatn, Tahoma, "Segoe UI", sans-serif; }
    a { color: __NEON__; text-decoration: none; }
    a:hover { color: __YELLOW__; }
    .en { direction: ltr; unicode-bidi: isolate; font-family: "Space Grotesk", ui-monospace, Consolas, monospace; }
  </style>
</helmet>

<div dir="rtl" style="position: relative; width: 2245px; height: 3178px; background: __BG__; color: __TEXT__; overflow: hidden; font-family: Vazirmatn, Tahoma, sans-serif;">

  <div style="position: absolute; inset: 0; overflow: hidden;">
    <div style="position: absolute; left: -300px; top: 620px; width: 3000px; height: 210px; transform: rotate(-24deg); background: linear-gradient(90deg, rgba(43,234,124,0) 0%, rgba(43,234,124,0.10) 45%, rgba(43,234,124,0) 100%);"></div>
    <div style="position: absolute; left: -400px; top: 1500px; width: 3000px; height: 130px; transform: rotate(18deg); background: linear-gradient(90deg, rgba(43,234,124,0) 0%, rgba(43,234,124,0.08) 50%, rgba(43,234,124,0) 100%);"></div>
    <div style="position: absolute; left: -200px; top: 2360px; width: 3000px; height: 260px; transform: rotate(-12deg); background: linear-gradient(90deg, rgba(43,234,124,0) 0%, rgba(43,234,124,0.07) 55%, rgba(43,234,124,0) 100%);"></div>
    <div style="position: absolute; left: 50%; bottom: 150px; margin-left: -700px; width: 1400px; height: 1200px; border-radius: 50%; background: radial-gradient(circle, rgba(43,234,124,0.16) 0%, rgba(43,234,124,0.05) 42%, rgba(43,234,124,0) 70%);"></div>
  </div>

  <div style="position: relative; z-index: 1; height: 100%; display: flex; flex-direction: column; padding: 54px 70px 44px 70px; box-sizing: border-box; gap: 34px;">

    <div style="display: flex; align-items: center; justify-content: space-between; gap: 50px;">
      <div style="display: flex; gap: 22px;">__TEAM__</div>
      <div style="display: flex; flex-direction: column; align-items: flex-end; gap: 16px; width: 880px; flex-shrink: 0;">
        <div style="display: flex; align-items: center; gap: 24px;">
          <div style="display: flex; flex-direction: column; align-items: flex-end; gap: 6px;">
            <div style="font-size: 88px; font-weight: 900; line-height: 1; color: __TEXT__; __GLOW__">سراتک</div>
            <div style="font-size: 26px; color: __MUTE__;">مجموعه مدارس سرای دانش</div>
          </div>
          <img src="saratech-logo.png" alt="SaraTech" style="width: 96px; height: auto; display: block;">
        </div>
        <div style="width: 100%; height: 2px; background: linear-gradient(to right, transparent, __NEONLINE__);"></div>
        <div class="en" style="font-size: 42px; font-weight: 400; letter-spacing: 1px; color: __TEXT__;">Junior Soccer Lightweight Primary</div>
      </div>
    </div>

    <div style="flex: 1; display: flex; flex-direction: column; gap: 36px; min-height: 0;">
      <div style="display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 40px; align-items: start;">
        <div style="display: flex; flex-direction: column; gap: 36px;">
          <div style="__PANEL__">__MECH__</div>
          <div style="__PANEL__">__ALGO__</div>
        </div>
        <div style="__PANEL__">__ELEC__</div>
      </div>
      <div style="flex: 1; position: relative; display: flex; align-items: center; justify-content: center; min-height: 0;">
        <img src="robot-primary.webp" alt="ربات فوتبالیست پرایمری سراتک" style="max-width: 900px; max-height: 100%; width: auto; height: auto; display: block; filter: drop-shadow(0 0 52px rgba(43,234,124,0.45));">
      </div>
    </div>

    <div style="display: flex; align-items: center; justify-content: space-between; gap: 46px; border-top: 2px solid __NEONLINE__; padding-top: 30px;">
      <div style="display: flex; align-items: center; gap: 20px; flex-shrink: 0;">
        <img src="saratech-logo.png" alt="SaraTech" style="width: 74px; height: auto; display: block;">
        <div style="background: #F4F7F5; border-radius: 12px; padding: 10px 14px; display: flex; align-items: center;">
          <img src="robocup-logo.jpg" alt="RoboCup" style="width: 156px; height: auto; display: block;">
        </div>
      </div>
      <div style="display: flex; flex-direction: column; gap: 8px; align-items: flex-end;">
        <div style="font-size: 38px; font-weight: 800; color: __TEXT__; line-height: 1.3;">مسابقات انتخابی ربوکاپ ۲۰۲۶ &mdash; اعزامی کره جنوبی</div>
        <div style="font-size: 29px; font-weight: 500; color: __YELLOW__;">تهران &mdash; اردیبهشت ۱۴۰۵</div>
      </div>
      <div class="en" style="display: flex; flex-direction: column; gap: 8px; align-items: flex-start;">
        <div style="font-size: 32px; font-weight: 700; color: __YELLOW__; line-height: 1.3;">Qualifier Tournament for<br>South Korea 2026 Championship</div>
        <div style="font-size: 27px; font-weight: 500; color: __TEXT__;">Tehran &middot; May 2026</div>
      </div>
    </div>

  </div>
</div>
</x-dc>

<script data-dc-script data-props='{"$preview":{"width":2245,"height":3178}}'>
class Component extends DCLogic {}
</script>
</body>
</html>
"""

for token, value in [
    ('__PANEL__', PANEL), ('__MECH__', mech), ('__ELEC__', elec), ('__ALGO__', algo),
    ('__TEAM__', team_html), ('__NEONLINE__', NEON_LINE), ('__NEON__', NEON),
    ('__YELLOW__', YELLOW), ('__TEXT__', TEXT), ('__MUTE__', MUTE), ('__BG__', BG),
    ('__GLOW__', GLOW),
]:
    HTML = HTML.replace(token, value)

if '__' in HTML.replace('__', '', 0) and '__' in HTML:
    leftovers = set(p for p in HTML.split('__')[1::2] if p.isupper())
    if leftovers:
        raise SystemExit('unsubstituted tokens: ' + ', '.join(sorted(leftovers)))

open('Main.dc.html', 'w', encoding='utf8').write(HTML)
print('Main.dc.html', len(HTML), 'chars')

canvas = {
    "artboards": [
        {"file": "Main.dc.html", "x": 0, "y": 0, "w": 2245, "h": 3178,
         "title": "پوستر ربات پرایمری", "print": "fixed"}
    ],
    "launch": {"view": "focused", "file": "Main.dc.html"}
}
open('canvas.json', 'w', encoding='utf8').write(json.dumps(canvas, ensure_ascii=False, indent=2))
print('canvas.json written')
