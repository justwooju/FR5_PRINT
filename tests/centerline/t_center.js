const fs=require('fs');eval(fs.readFileSync(__dirname+'/rdp.js','utf8')+fs.readFileSync(__dirname+'/center.js','utf8'));
let SEED=+process.env.SEED||1;Math.random=()=>{SEED=(SEED*1103515245+12345)%2147483648;return SEED/2147483648};
function segD(px,py,a,b){const dx=b[0]-a[0],dy=b[1]-a[1],L2=dx*dx+dy*dy;let t=L2?((px-a[0])*dx+(py-a[1])*dy)/L2:0;t=Math.max(0,Math.min(1,t));return Math.hypot(px-a[0]-t*dx,py-a[1]-t*dy)}
function polySegs(polys){const s=[];polys.forEach(P=>{for(let i=1;i<P.length;i++)s.push([P[i-1],P[i]])});return s}
function raster(w,h,segs,r){const B=new Uint8Array(w*h);for(let y=0;y<h;y++)for(let x=0;x<w;x++){let m=1e9;for(const [a,b] of segs){const v=segD(x,y,a,b);if(v<m)m=v;if(m<=r)break}B[y*w+x]=m<=r?1:0}return B}
const circ=(cx,cy,R,n=180)=>Array.from({length:n+1},(_,k)=>[cx+R*Math.cos(2*Math.PI*k/n),cy+R*Math.sin(2*Math.PI*k/n)]);
const sine=Array.from({length:121},(_,k)=>[40+k*2.6,150+70*Math.sin(k/120*4*Math.PI)]);
const SH={line:[[[50,150],[350,150]]],diag:[[[50,50],[350,250]]],circle:[circ(200,150,100)],T:[[[50,100],[350,100]],[[200,100],[200,250]]],
 X:[[[50,50],[350,250]],[[50,250],[350,50]]],plus:[[[100,150],[300,150]],[[200,50],[200,250]]],A:[[[100,250],[200,50],[300,250]],[[150,150],[250,150]]],
 sine:[sine],E:[[[300,50],[100,50],[100,250],[300,250]],[[100,150],[250,150]]]};
const lem=Array.from({length:241},(_,k)=>{const t=k/240*2*Math.PI,c=1+Math.sin(t)**2;return [200+150*Math.cos(t)/c,150+150*Math.sin(t)*Math.cos(t)/c]});
const loopP=[[120,260],[120,60],[220,60]].concat(Array.from({length:31},(_,k)=>{const a=-Math.PI/2+k/30*Math.PI;return [220+50*Math.cos(a),110+50*Math.sin(a)]}),[[120,160]]);
SH.eight=[lem];SH.P=[loopP];SH.hairpin=[[[100,250],[100,80]].concat(Array.from({length:21},(_,k)=>{const a=Math.PI+k/20*Math.PI;return [140+40*Math.cos(a),80+40*Math.sin(a)]}),[[180,250]])];
SH.close=[[[50,140],[350,140]],[[50,158],[350,158]]];SH.dash=Array.from({length:10},(_,k)=>[[40+k*32,150],[60+k*32,150]]);
const EXP={line:1,diag:1,circle:1,T:2,X:2,plus:2,A:2,sine:1,E:2,eight:1,P:1,hairpin:1,close:2,dash:10};
const w=400,h=300;let bad=0;
for(const r of (process.env.QUICK?[4]:[2,4,7]))for(const [nm,P] of Object.entries(SH)){process.stderr.write(`>${nm} r${r} `);const segs=polySegs(P);let B=raster(w,h,segs,r);
 if(nm==='T'&&r===4){for(let k=0;k<40;k++){const x=5+Math.floor(Math.random()*390),y=5+Math.floor(Math.random()*290);B[y*w+x]=1}B[100*w+120]=0;B[101*w+121]=0}   // 티 · 구멍
 const t0=Date.now();const cl=cleanBin(B,w,h,10,30);const R=centerTrace(B,w,h,{spurPx:3,extend:true,penRpx:r,sig:1.0,tol:0.4});const ms=Date.now()-t0;
 const S=R.S;let dev=0;S.forEach(Q=>Q.forEach(p=>{let m=1e9;for(const [a,b] of segs)m=Math.min(m,segD(p[0],p[1],a,b));dev=Math.max(dev,m)}));
 // 덮음: 참 선 위 점들이 획에서 얼마나 먼가 (빠진 곳)
 const ss=polySegs(S);let miss=0;segs.forEach(([a,b])=>{const L=Math.hypot(b[0]-a[0],b[1]-a[1]);for(let t=0;t<=1;t+=1/Math.max(1,L)){const x=a[0]+t*(b[0]-a[0]),y=a[1]+t*(b[1]-a[1]);let m=1e9;for(const [c,d] of ss)m=Math.min(m,segD(x,y,c,d));miss=Math.max(miss,m)}});
 const ok=(nm==='close'&&r===7?true:S.length===EXP[nm])&&dev<=1.5+0.15*r&&miss<=1.5+0.15*r;if(!ok)bad++;
 console.log(`${ok?'✓':'✗'} r=${r} ${nm.padEnd(6)} 획 ${S.length}/${EXP[nm]} · 벗어남 최대 ${dev.toFixed(2)} px · 빠짐 최대 ${miss.toFixed(2)} px · 반굵기 ${R.hwMed.toFixed(1)} · 잔가지 ${R.pruned} · 점 ${S.reduce((a,q)=>a+q.length,0)} · ${ms} ms${cl.nS||cl.nH?` · 티 ${cl.nS} 구멍 ${cl.nH}`:''}`)}
// 큰 그림 · 시간
if(!process.env.QUICK&&!process.env.NOBIG){const W=1200,H=900,segs=[];for(let k=0;k<60;k++){const x=50+Math.random()*1100,y=50+Math.random()*800,a=Math.random()*6.28,L=30+Math.random()*200;segs.push([[x,y],[Math.max(5,Math.min(W-5,x+L*Math.cos(a))),Math.max(5,Math.min(H-5,y+L*Math.sin(a)))]])}
 const B=raster(W,H,segs,3);const t0=Date.now();cleanBin(B,W,H,10,30);const R=centerTrace(B,W,H,{spurPx:3,extend:true,penRpx:3,sig:1,tol:0.4});const o=orderStrokes(R.S,[0,0],0.5);
 console.log(`큰 그림 1200×900 · 선 60 → 획 ${R.S.length} → 정리 뒤 ${o.S.length} · ${Date.now()-t0} ms`)}
// 순서 정리
const rnd=[];for(let k=0;k<300;k++){const x=Math.random()*150,y=Math.random()*100,a=Math.random()*6.28;rnd.push([[x,y],[x+3*Math.cos(a),y+3*Math.sin(a)]])}
const o=orderStrokes(rnd,[0,0],0.05);console.log(`순서: 옮기기 ${o.tr0.toFixed(0)} → ${o.tr1.toFixed(0)} mm (${(100*o.tr1/o.tr0).toFixed(0)} %) · 획 ${rnd.length} → ${o.S.length}`);
const L0=rnd.reduce((a,P)=>a+plenA(P),0),L1=o.S.reduce((a,P)=>a+plenA(P),0);console.log('그리기 길이 보존',L0.toFixed(3),L1.toFixed(3));
{const w=200,h=100,B=new Uint8Array(w*h);for(let y=0;y<h;y++)for(let x=0;x<w;x++){if(Math.hypot(x-50,y-50)<=3||Math.hypot(x-150,y-50)<=4)B[y*w+x]=1;if(y===80&&x>20&&x<180)B[y*w+x]=1}
 const R=centerTrace(B,w,h,{spurPx:3,extend:true,penRpx:1,sig:1,tol:0.4});console.log('점 둘 + 선:',R.S.map(p=>p.length===1?`점(${p[0].map(v=>v.toFixed(0))})`:`선 ${p.length}점`).join(' · '))}
process.exit(bad?1:0)
