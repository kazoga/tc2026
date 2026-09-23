"""参照ガイドをA5縦・日本語PDFへ変換する。依存: reportlab。"""
from pathlib import Path
from xml.sax.saxutils import escape
import re
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from reportlab.lib.pagesizes import A5
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle

ROOT = Path(__file__).resolve().parent
pdfmetrics.registerFont(UnicodeCIDFont('HeiseiKakuGo-W5'))
body = ParagraphStyle('body', fontName='HeiseiKakuGo-W5', fontSize=10, leading=16,
                      wordWrap='CJK', spaceAfter=6)
heading = ParagraphStyle('heading', parent=body, fontSize=14, leading=21,
                         spaceBefore=14, spaceAfter=9, keepWithNext=True, textColor=colors.HexColor('#153f5a'))
small = ParagraphStyle('small', parent=body, fontSize=8, leading=12)
def rich(text):
    text=escape(text)
    text=re.sub(r'\[([^\]]+)\]\((https://[^)]+)\)', r'<link href="\2" color="#12699b">\1</link>',text)
    text=re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'\1',text)
    text=re.sub(r'\*\*([^*]+)\*\*',r'<b>\1</b>',text)
    return text.replace('`','')
def footer(canvas, doc):
    canvas.setFont('HeiseiKakuGo-W5',8)
    canvas.drawString(30,18,'TC2026 / 走行システムの機能と制約')
    canvas.drawRightString(A5[0]-30,18,str(doc.page))
flow=[];lines=(ROOT/'README.md').read_text().splitlines();i=0
while i<len(lines):
    line=lines[i];i+=1
    if not line.strip():continue
    if line.startswith('|'):
        rows=[line]
        while i<len(lines) and lines[i].startswith('|'):rows.append(lines[i]);i+=1
        cells=[[Paragraph(rich(c.strip()),small) for c in r.strip('|').split('|')] for r in rows if not re.match(r'^\|[\s:|\-]+$',r)]
        t=Table(cells,colWidths=[(A5[0]-60)*.29,(A5[0]-60)*.71],repeatRows=1,hAlign='LEFT')
        t.setStyle(TableStyle([('VALIGN',(0,0),(-1,-1),'TOP'),('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e5eef4')),('GRID',(0,0),(-1,-1),.3,colors.HexColor('#c1ccd3')),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]));flow.extend([t,Spacer(1,8)]);continue
    if line.startswith('#'):
        flow.append(Paragraph(rich(line.lstrip('# ')),heading));continue
    if line.startswith('- '):
        line='・'+line[2:]
    else:
        # Markdownのソフト改行を段落内でつなぎ、紙幅に合わせて折り返す。
        paragraph=[line]
        while i<len(lines) and lines[i].strip() and not lines[i].startswith(('#','|','- ')):
            paragraph.append(lines[i]);i+=1
        line=' '.join(paragraph)
    flow.append(Paragraph(rich(line),body))
output=ROOT/'output/pdf/tc2026_software_review.pdf';output.parent.mkdir(parents=True,exist_ok=True)
SimpleDocTemplate(str(output),pagesize=A5,rightMargin=30,leftMargin=30,topMargin=25,bottomMargin=35,
                  title='走行システムの機能と制約',author='tc2026 repository').build(flow,onFirstPage=footer,onLaterPages=footer)
print(output)
