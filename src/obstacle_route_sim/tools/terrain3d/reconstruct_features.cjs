#!/usr/bin/env node
// 提供版の OSM 解釈と航空写真樹冠解析をサーバー側でも同じ条件で使う。
const fs=require('node:fs'), vm=require('node:vm'), path=require('node:path');
const [mode,input,output]=process.argv.slice(2);
const html=fs.readFileSync(path.join(__dirname,'terrain3d_v0_4_1.html'),'utf8');
for(const n of ['core','geo','recon'])vm.runInThisContext(html.split('/*@'+n+'*/')[1].split('/*@end'+n+'*/')[0]);
const cfg=JSON.parse(fs.readFileSync(input,'utf8'));
if(mode==='query')fs.writeFileSync(output,TerrainGeo.overpassQuery(cfg.bbox));
else if(mode==='features'){
 const osm=JSON.parse(fs.readFileSync(cfg.osm,'utf8'));
 const features=TerrainGeo.osmToFeatures(osm,cfg.origin);
 const mask=TerrainRecon.pixelMask(features,cfg.photoWidth,cfg.photoHeight,cfg.width,cfg.depth);
 const rgba=new Uint8Array(fs.readFileSync(cfg.rgba));
 const trees=TerrainRecon.detectTrees(rgba,cfg.photoWidth,cfg.photoHeight,
   cfg.width/cfg.photoWidth,cfg.width,cfg.depth,{mask,maskW:cfg.photoWidth,maskH:cfg.photoHeight,
   sensitivity:cfg.sensitivity||'balanced',minDist:2,cap:10000});
 fs.writeFileSync(output,JSON.stringify({features,trees,treeStats:trees.stats}));
 fs.writeFileSync(path.join(path.dirname(output),'exclusion_mask.raw'),mask);
}else throw Error('mode は query または features');
