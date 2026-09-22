"""CAD形状を平行投影し、取付方向が分かる静止画を生成する。"""
from pathlib import Path
import math
import numpy as np
import cadquery as cq
from PIL import Image, ImageDraw, ImageFont


def render(parts, output: Path, angle: float, offset: float) -> None:
    parts = dict(parts)
    # 締結説明用の簡略ねじ頭と、真上から挿入する工具の包絡。
    parts['frame_bolts'] = cq.Compound.makeCompound([
        cq.Workplane('XY', origin=(x, 0, 8)).circle(5).extrude(1).union(
            cq.Workplane('XY', origin=(x, 0, 9)).circle(4.25).extrude(5)).val()
        for x in (-85, 85)])
    parts['tool_envelope'] = cq.Compound.makeCompound([
        cq.Workplane('XY', origin=(x, 0, 16)).circle(8).extrude(70).val()
        for x in (-85, 85)])
    colors={'bracket':(32,142,156), 'aluminum_plate':(193,204,218),
            'sensor_envelope':(63,74,89),'frame_envelope':(145,157,173),
            'frame_bolts':(235,130,35),'tool_envelope':(75,185,110)}
    font_path='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
    if not Path(font_path).exists():
        font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=lambda size:ImageFont.truetype(font_path,size)
    meshes={}
    for name,shape in parts.items():
        vertices, triangles=shape.tessellate(.2)
        meshes[name]=(np.array([v.toTuple() for v in vertices]),triangles)
    for view in ['assembly','side','printed_part','fastening']:
        im=Image.new('RGB',(1600,1200),(246,248,252));d=ImageDraw.Draw(im)
        if view=='side':
            right=np.array([0,1,0.]);up=np.array([0,0,1.]);depth=np.array([1,0,0.])
        else:
            depth=np.array([1.25,1.65,1.15]);depth/=np.linalg.norm(depth)
            right=np.cross(np.array([0,0,1.]),depth);right/=np.linalg.norm(right)
            up=np.cross(depth,right)
        selected = {k:v for k,v in meshes.items() if k not in ('frame_bolts','tool_envelope')}
        if view == 'printed_part':
            selected = {'bracket':meshes['bracket']}
        elif view == 'fastening':
            selected = meshes
        arrays=np.concatenate([v for v,t in selected.values()])
        xy=np.stack([arrays@right,arrays@up],axis=1)
        low,high=xy.min(axis=0),xy.max(axis=0)
        scale=min(1320/(high[0]-low[0]),750/(high[1]-low[1]))
        center=(low+high)/2
        def project(v):
            return (800+((v@right)-center[0])*scale,610-((v@up)-center[1])*scale)
        pixels=np.array(im)
        zbuffer=np.full((1200,1600),-np.inf)
        for name,(vertices,triangles) in selected.items():
            for tri in triangles:
                pts=vertices[list(tri)]
                normal=np.cross(pts[1]-pts[0],pts[2]-pts[0]);length=np.linalg.norm(normal)
                if length<1e-9:continue
                normal/=length
                shade=.65+.35*abs(float(normal@np.array([.3,.4,.866])))
                color=tuple(int(c*shade) for c in colors[name])
                uv=np.array([project(v) for v in pts]);z=pts@depth
                x0=max(0,int(np.floor(uv[:,0].min())));x1=min(1599,int(np.ceil(uv[:,0].max())))
                y0=max(0,int(np.floor(uv[:,1].min())));y1=min(1199,int(np.ceil(uv[:,1].max())))
                if x1<x0 or y1<y0:continue
                a,b,c=uv
                den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
                if abs(den)<1e-9:continue
                yy,xx=np.mgrid[y0:y1+1,x0:x1+1];xx=xx+.5;yy=yy+.5
                wa=((b[1]-c[1])*(xx-c[0])+(c[0]-b[0])*(yy-c[1]))/den
                wb=((c[1]-a[1])*(xx-c[0])+(a[0]-c[0])*(yy-c[1]))/den
                wc=1-wa-wb
                zz=wa*z[0]+wb*z[1]+wc*z[2]
                target=zbuffer[y0:y1+1,x0:x1+1]
                mask=(wa>=-1e-8)&(wb>=-1e-8)&(wc>=-1e-8)&(zz>target)
                target[mask]=zz[mask]
                pixels[y0:y1+1,x0:x1+1][mask]=color
        im=Image.fromarray(pixels);d=ImageDraw.Draw(im)
        title={'assembly':'MID-360 / 25°前下がりブラケット',
               'side':'側面：50 mm前方へ移動 / 前下がり25°',
               'printed_part':'3Dプリント部品：一体ブラケット',
               'fastening':'フレーム固定ねじ：外側2か所を真上から締付'}[view]
        d.text((75,45),title,font=font(48),fill=(23,40,62))
        d.text((75,115),'JTABS 150×60×2 mm対応 ｜ HFS5横梁・上面溝/M5固定',font=font(26),fill=(71,84,102))
        if view=='fastening':
            d.text((75,1010),'緑：直径16 mmの工具包絡　橙：M5ねじ頭・ワッシャー',font=font(28),fill=(23,40,62))
            d.text((75,1060),'板幅150 mm ／ 穴ピッチ170 mm ／ 板端と工具外周の隙間2 mm',font=font(28),fill=(23,40,62))
        elif view=='side':
            # 横梁中心と傾斜支持面中心の水平距離を実座標で寸法表示。
            p0=project(np.array([0,0,-27.]));p1=project(np.array([0,offset,-27.]))
            d.line((*p0,*p1),fill=(219,104,48),width=5)
            for p in (p0,p1):
                d.line((p[0],p[1]-12,p[0],p[1]+12),fill=(219,104,48),width=4)
            d.text(((p0[0]+p1[0])/2-65,p0[1]+15),f'{offset:g} mm',font=font(32),fill=(148,60,20))
            d.line((1060,990,1450,990),fill=(219,104,48),width=8)
            d.polygon([(1450,990),(1418,973),(1418,1007)],fill=(219,104,48))
            d.text((1140,1020),'ロボット前方',font=font(30),fill=(148,60,20))
            d.text((80,1020),'横梁は紙面の奥行方向',font=font(30),fill=(40,55,74))
        elif view=='assembly':
            labels=[('印刷ブラケット',colors['bracket']),('既存JTABS 150×60×2 mm',colors['aluminum_plate']),('MID-360簡略外形',colors['sensor_envelope'])]
            for i,(label,color) in enumerate(labels):
                x=75+i*500
                d.rectangle((x,1045,x+25,1070),fill=color)
                d.text((x+40,1035),label,font=font(26),fill=(23,40,62))
        else:
            d.text((75,1040),'底面を下に印刷。上桟の下面はサポート要。単位：mm',font=font(28),fill=(23,40,62))
        d.text((75,1140),'設計試作 v3・前方50 mm ｜ 実機適合・強度未確認。既存板は放熱推奨条件未達',font=font(23),fill=(88,102,117))
        im.save(output/(view+'.png'))
