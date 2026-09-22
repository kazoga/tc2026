// ブラウザと同じコアを読み込み、描画に依存せず観測の意味を検証する。
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[2],'utf8');
for(const name of ['core','geo','recon','world','experiments']){
  vm.runInThisContext(html.split(`/*@${name}*/`)[1].split(`/*@end${name}*/`)[0]);
}
const E=TerrainExperiments,W=TerrainWorld,C=TerrainCore;
const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-8,`${a} != ${b}`);
const f=E.lidarDirection(0,0,25,0),back=E.lidarDirection(Math.PI,0,25,0);
close(f[2],-Math.sin(25*Math.PI/180));close(back[2],-f[2]);
close(E.lidarDirection(Math.PI/2,0,25,0)[2],0);
close(Math.hypot(...f),1);
const turned=E.lidarDirection(0,0,25,Math.PI/2);
close(turned[0],-f[1]);close(turned[2],f[2]);
assert.throws(()=>E.lidarSettings({sensors:{lidarPitchDownDeg:NaN}}));
assert.throws(()=>E.lidarSettings({}, {minPitch:30,maxPitch:10}));
assert.equal(E.lidarSettings({sensors:{pitchDeg:-25}}).mountPitchDownDeg,25);
assert.equal(E.lidarSettings({sensors:{pitchDeg:-25,lidarPitchDownDeg:0}}).mountPitchDownDeg,0);
const scene=C.normalize({app:'terrain3d',version:1,unit:'m',up:'Z',region:{w:40,d:40,res:.5},
 source:{kind:'contour',contour:{base:0,noise:0,lines:[]}},sea:{level:-10},objects:[],
 vegetation:{trees:[]},robots:[{id:'r',type:'ugv',x:0,y:0,yaw:0,
 sensors:{height:.6,lidarRange:8,lidarPitchDownDeg:0}}]});
const world=W.compile(scene),robot=scene.robots[0],opts={channels:16,columns:180,maxRange:8};
// 地形生成器の海岸・起伏補正を除き、解析解と比較できる平面にする。
world.grid.height.fill(0);world.grid.valid.fill(1);world.grid.cls.fill(C.CI.road);
world.obstacles=[];world.trees=[];
const truth=Array.from(W.traversability(world,scene,robot).mask);
const flat=E.scan(world,scene,robot,opts);
robot.sensors.lidarPitchDownDeg=25;
const down=E.scan(world,scene,robot,opts);
assert.deepEqual(Array.from(W.traversability(world,scene,robot).mask),truth);
const a=flat.points[flat.rays[0].pointIndex],b=down.points[down.rays[0].pointIndex];
assert.ok(Math.abs(a[1]-.6/Math.tan(7*Math.PI/180))<.02);
assert.ok(Math.abs(b[1]-.6/Math.tan(32*Math.PI/180))<.02);
assert.equal(down.rays[90].range,null); // 後方の最下段ビームは上を向く。
const m0=E.observedTraversability(world,scene,robot,flat),m25=E.observedTraversability(world,scene,robot,down);
assert.notDeepEqual(m0.observedGround,m25.observedGround);
assert.notDeepEqual(m0.mask,m25.mask);
for(let i=0;i<m25.mask.length;i++)if(m25.mask[i]===1){assert.equal(m25.observedGround[i],1);assert.equal(truth[i],1);}
const up=E.scan(world,scene,robot,{...opts,lidarPitchDownDeg:0,minPitch:70,maxPitch:80});
const empty=E.observedTraversability(world,scene,robot,up);
assert.ok(empty.mask.every(v=>v===-1)); // 上空や無反射を地面の空きにしない。
const occluded=E.observedTraversability(world,scene,robot,{...down,
 rays:down.rays.map(r=>({...r,entityId:'wall'}))});
assert.ok(occluded.mask.every(v=>v===-1)); // 障害物への反射を地面観測にしない。
console.log(JSON.stringify({flat:m0.summary,down25:m25.summary}));
