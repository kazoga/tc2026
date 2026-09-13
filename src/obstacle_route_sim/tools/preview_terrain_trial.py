#!/usr/bin/env python3
"""Gazebo 真値軌跡を添付 terrain3d 上で再生する HTML を生成する."""

import argparse
import csv
import json
from pathlib import Path


def generate(directory: Path) -> None:
    """計測結果を埋め込み、推定経路と実測軌跡の混同を防ぐ表示を付ける."""
    with (directory/'trajectory.csv').open() as stream:
        rows = [{k: float(v) for k,v in row.items()} for row in csv.DictReader(stream)]
    if not rows:
        raise ValueError('真値軌跡がないため再生 HTML を作成できない')
    scene = json.loads((directory/'scene.json').read_text())
    scene['robots'][0]['path'] = [[r['x'],r['y']] for r in rows]
    html = (Path(__file__).parent/'terrain3d/terrain3d_v0_4_1.html').read_text()
    embedded = json.dumps(scene,ensure_ascii=False).replace('<','\\u003c')
    html = html.replace('<script type="application/json" id="embedded-scene"></script>',
                        '<script type="application/json" id="embedded-scene">'+embedded+'</script>')
    replay = '''<div style="position:fixed;bottom:18px;left:22%;right:12%;z-index:9999;
background:#182231;color:white;padding:12px;border:1px solid #5d8a99;border-radius:8px">
<b>Gazebo 真値軌跡の再生（ライブ制御ではありません）</b>
<p style="margin:4px 0">合成通路・理想自己位置。表示車体は簡略図形です。</p>
<button id="trial-play">再生／停止</button><span id="trial-time"></span>
<input id="trial-frame" type="range" min="0" value="0" style="width:100%"></div>
<script>
const trialRows=ROWS;
let trialPlaying=false, trialIndex=0;
document.getElementById('trial-frame').max=trialRows.length-1;
document.getElementById('trial-play').onclick=()=>trialPlaying=!trialPlaying;
function trialShow(index){
 if(!window.TerrainApp || typeof robGrp==='undefined' || !robGrp.children.length)return;
 const row=trialRows[index],robot=robGrp.children[0];
 robot.position.set(row.x,row.y,row.z);
 robot.rotation.z=row.yaw-Math.PI/2;
 dirty3=true;
 document.getElementById('trial-time').textContent=
   ' シミュレーション '+row.sim_s.toFixed(1)+' 秒';
 document.getElementById('trial-frame').value=index;
}
document.getElementById('trial-frame').oninput=e=>{
 trialIndex=+e.target.value;trialShow(trialIndex);
};
setInterval(()=>{if(trialPlaying){trialIndex=(trialIndex+1)%trialRows.length;trialShow(trialIndex);}},100);
</script>'''.replace('ROWS',json.dumps(rows))
    result = json.loads((directory/'result.json').read_text())
    if scene['source']['kind'] == 'geo' or result.get('localization_mode') == 'gnss':
        description = ('国土地理院 DEM・OSM の現地小区間' if scene['source']['kind'] == 'geo'
                       else '合成通路')
        description += '／仮想 GNSS 経由。表示車体は簡略図形。現地実測で未校正。'
        replay = replay.replace('合成通路・理想自己位置。表示車体は簡略図形です。',description)
    html = html.replace('</body>',replay+'</body>')
    (directory/'replay.html').write_text(html,encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    generate(parser.parse_args().directory)
