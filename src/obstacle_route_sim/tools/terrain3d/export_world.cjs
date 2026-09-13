#!/usr/bin/env node
// 添付版の正規地形コンパイラと SDF exporter を UI 非依存で使用する。
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const crypto = require('node:crypto');

const [input, output] = process.argv.slice(2);
if (!input || !output) throw Error('使用法: node export_world.cjs scene.json output_directory');
const html = fs.readFileSync(path.join(__dirname, 'terrain3d_v0_4_1.html'), 'utf8');
for (const name of ['core', 'geo', 'recon', 'world']) {
  const start = `/*@${name}*/`, end = `/*@end${name}*/`;
  if (!html.includes(start) || !html.includes(end)) throw Error(`未対応の HTML: ${name}`);
  vm.runInThisContext(html.split(start)[1].split(end)[0], {filename: name});
}
vm.runInThisContext('function sdfUnified' + html.split('function sdfUnified')[1]
  .split('function usdUnified')[0]);
const scene = TerrainCore.normalize(JSON.parse(fs.readFileSync(input, 'utf8')));
const world = TerrainWorld.compile(scene);
const g = world.grid;
if (Array.from(g.valid).some(v => !v)) throw Error('欠測標高のある地形は出力しない');
const positions = [];
for (let y = 0; y < g.h - 1; y++) for (let x = 0; x < g.w - 1; x++) {
  for (const [a, b] of [[x,y],[x+1,y],[x+1,y+1],[x,y],[x+1,y+1],[x,y+1]]) {
    positions.push(g.ox+a*g.res, g.oy+b*g.res, g.height[b*g.w+a]);
  }
}
const meshes = [{name:'terrain', positions, collision:true}];
if (scene.source.kind === 'geo') {
  globalThis.THREE = require('three');
  globalThis.TC = TerrainCore;
  globalThis.TW = TerrainWorld;
  globalThis.WORLD = world;
  globalThis.S = scene;
  globalThis.window = {};
  vm.runInThisContext('function buildingTriangles'+html.split('function buildingTriangles')[1]
    .split('function geometryFromPositions')[0]);
  vm.runInThisContext('function boxPositions'+html.split('function boxPositions')[1]
    .split('function terrainSurface')[0]);
  meshes.push(...worldMeshes(g, {x:0,y:0,z:0}));
}
// 試験シーンの直方体を、正規コンパイラが確定した polygon から押し出す。
for (const o of (scene.source.kind === 'geo' ? [] : world.obstacles)) {
  if (o.source === 'tree' || o.source === 'canopy-proxy') continue;
  if (o.type !== 'polygon' || o.poly.length !== 4 || o.holes?.length) {
    throw Error(`未対応障害物 ${o.id}: ${o.type}。直方体・単木のみ対応`);
  }
  const verts = [o.z0,o.z1].flatMap(z => o.poly.map(p => [p[0],p[1],z]));
  const faces = [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
    [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]];
  meshes.push({name:'m_'+TerrainWorld.hash(o.id), collision:true,
    positions:faces.flatMap(f => f.flatMap(i => verts[i]))});
}
fs.mkdirSync(output, {recursive:true});
for (const m of meshes) {
  const lines = [];
  for (let i=0;i<m.positions.length;i+=3) lines.push('v '+m.positions.slice(i,i+3).join(' '));
  // DART の mesh collision は各頂点の法線を必要とする。
  for (let i=0;i<m.positions.length;i+=9) {
    const p=m.positions, u=[p[i+3]-p[i],p[i+4]-p[i+1],p[i+5]-p[i+2]],
      v=[p[i+6]-p[i],p[i+7]-p[i+1],p[i+8]-p[i+2]];
    const n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
    const length=Math.hypot(...n);
    if (length<1e-12) throw Error('退化三角形は物理世界に出力しない');
    for(let k=0;k<3;k++) lines.push('vn '+n.map(x=>x/length).join(' '));
  }
  for (let i=1;i<=m.positions.length/3;i+=3) {
    lines.push('f '+[i,i+1,i+2].map(k=>`${k}//${k}`).join(' '));
  }
  fs.writeFileSync(path.join(output,m.name+'.obj'),lines.join('\n')+'\n');
}
fs.writeFileSync(path.join(output,'environment.sdf'),
  sdfUnified(g,{x:0,y:0,z:0},{tree:'trunk'},meshes,world.trees)
    .replace(/<collision name="(trunk|canopy)">/g,'<collision name="$1_collision">'));
fs.writeFileSync(path.join(output,'world.json'),JSON.stringify({
  worldId:world.id, frames:world.frames, trees:world.trees, obstacles:world.obstacles,
  meshes:meshes.map(({name,id,kind})=>({name,id,kind})),
  grid:{w:g.w,h:g.h,res:g.res,ox:g.ox,oy:g.oy,height:Array.from(g.height)},
  terrain3dSha256:crypto.createHash('sha256').update(html).digest('hex'),
  limitation:scene.source.kind === 'geo' ? '公開地理データ由来。未観測の段差や地物は再現しない。' : '実測ではない合成試験環境。直方体と単木の静的衝突形状。'
},null,2));
console.log(JSON.stringify({worldId:world.id,meshes:meshes.length,trees:world.trees.length}));
