// 무작위 선 그림으로 멈춤 · NaN 찾기 (씨앗 고정)
const fs=require('fs');eval(fs.readFileSync(__dirname+'/rdp.js','utf8')+fs.readFileSync(__dirname+'/center.js','utf8'));
let seed=+process.argv[2]||1;const rnd=()=>{seed=(seed*1103515245+12345)%2147483648;return seed/2147483648};
function segD(px,py,a,b){const dx=b[0]-a[0],dy=b[1]-a[1],L2=dx*dx+dy*dy;let t=L2?((px-a[0])*dx+(py-a[1])*dy)/L2:0;t=Math.max(0,Math.min(1,t));return Math.hypot(px-a[0]-t*dx,py-a[1]-t*dy)}
const W=300,H=220,segs=[];const ns=3+Math.floor(rnd()*12);for(let k=0;k<ns;k++){const x=10+rnd()*280,y=10+rnd()*200,a=rnd()*6.28,L=10+rnd()*150;segs.push([[x,y],[Math.max(3,Math.min(W-3,x+L*Math.cos(a))),Math.max(3,Math.min(H-3,y+L*Math.sin(a)))]])}
const r=1+rnd()*6,B=new Uint8Array(W*H);for(let y=0;y<H;y++)for(let x=0;x<W;x++){let m=1e9;for(const [a,b] of segs)m=Math.min(m,segD(x,y,a,b));B[y*W+x]=m<=r?1:0}
for(let k=0;k<30;k++){const i=Math.floor(rnd()*W*H);B[i]=1-B[i]}
const t0=Date.now();cleanBin(B,W,H,6,20);const R=centerTrace(B,W,H,{spurPx:3,extend:true,penRpx:r*rnd(),sig:1,tol:0.4});const o=orderStrokes(R.S,[0,0],0.5);
const nan=o.S.some(P=>P.some(p=>!isFinite(p[0])||!isFinite(p[1])));console.log(`seed ${process.argv[2]} 선 ${ns} r ${r.toFixed(1)} → 획 ${R.S.length}/${o.S.length} · ${Date.now()-t0} ms${nan?' · NaN!':''}`);process.exit(nan?2:0)
