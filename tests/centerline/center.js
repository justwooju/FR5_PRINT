// ── v0.2 중심선 · 정리 (브라우저 안): 티 지우기 · 구멍 메우기 → 골격(Zhang–Suen + 계단 지우기) → 그래프 → 잔가지 버림
//    → 갈림길에서 곧게 이어지는 것끼리 잇기 → 끝 늘리기 → 매끈 · 줄이기 → 순서 정리(가까운 것부터 · 2-opt · 작은 틈 잇기)
function ccLabel(B,w,h,val,c8){const lab=new Int32Array(w*h).fill(-1),area=[],touch=[],st=new Int32Array(w*h);let n=0;
 for(let i=0;i<w*h;i++){if(B[i]!==val||lab[i]>=0)continue;let sp=0,a=0,t=false;st[sp++]=i;lab[i]=n;
  while(sp){const j=st[--sp];a++;const x=j%w,y=(j-x)/w;if(!x||!y||x===w-1||y===h-1)t=true;
   for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){if((!dx&&!dy)||(!c8&&dx&&dy))continue;const X=x+dx,Y=y+dy;if(X<0||Y<0||X>=w||Y>=h)continue;const k=Y*w+X;if(B[k]===val&&lab[k]<0){lab[k]=n;st[sp++]=k}}}
  area.push(a);touch.push(t);n++}return {lab,n,area,touch}}
function cleanBin(B,w,h,minA,holeA){let nS=0,nH=0;
 if(minA>0){const L=ccLabel(B,w,h,1,true);for(let i=0;i<w*h;i++)if(B[i]&&L.area[L.lab[i]]<minA)B[i]=0;nS=L.area.filter(a=>a<minA).length}
 if(holeA>0){const L=ccLabel(B,w,h,0,false);for(let i=0;i<w*h;i++)if(!B[i]&&!L.touch[L.lab[i]]&&L.area[L.lab[i]]<holeA)B[i]=1;nH=L.area.filter((a,k)=>!L.touch[k]&&a<holeA).length}
 return {nS,nH}}
function distT(B,w,h){const D=new Float32Array(w*h),R2=Math.SQRT2;for(let i=0;i<w*h;i++)D[i]=B[i]?1e9:0;
 for(let y=0;y<h;y++)for(let x=0;x<w;x++){const i=y*w+x;if(!D[i])continue;let v=D[i];if(x>0)v=Math.min(v,D[i-1]+1);else v=Math.min(v,1);
  if(y>0){v=Math.min(v,D[i-w]+1);if(x>0)v=Math.min(v,D[i-w-1]+R2);if(x<w-1)v=Math.min(v,D[i-w+1]+R2)}else v=Math.min(v,1);D[i]=v}
 for(let y=h-1;y>=0;y--)for(let x=w-1;x>=0;x--){const i=y*w+x;if(!D[i])continue;let v=D[i];if(x<w-1)v=Math.min(v,D[i+1]+1);else v=Math.min(v,1);
  if(y<h-1){v=Math.min(v,D[i+w]+1);if(x<w-1)v=Math.min(v,D[i+w+1]+R2);if(x>0)v=Math.min(v,D[i+w-1]+R2)}else v=Math.min(v,1);D[i]=v}return D}
function thinZS(B,w,h){const S=Uint8Array.from(B),P=(x,y)=>(x<0||y<0||x>=w||y>=h)?0:S[y*w+x],del=[];let ch=true,it=0;
 while(ch&&it<1000){ch=false;it++;for(let ps=0;ps<2;ps++){del.length=0;
  for(let y=0;y<h;y++)for(let x=0;x<w;x++){const i=y*w+x;if(!S[i])continue;
   const p2=P(x,y-1),p3=P(x+1,y-1),p4=P(x+1,y),p5=P(x+1,y+1),p6=P(x,y+1),p7=P(x-1,y+1),p8=P(x-1,y),p9=P(x-1,y-1),b=p2+p3+p4+p5+p6+p7+p8+p9;if(b<2||b>6)continue;
   const a=(!p2&&p3)+(!p3&&p4)+(!p4&&p5)+(!p5&&p6)+(!p6&&p7)+(!p7&&p8)+(!p8&&p9)+(!p9&&p2);if(a!==1)continue;
   if(ps===0?(p2&&p4&&p6)||(p4&&p6&&p8):(p2&&p4&&p8)||(p2&&p6&&p8))continue;del.push(i)}
  if(del.length){ch=true;for(const i of del)S[i]=0}}}
 // 계단 · 겹친 화소: 이웃이 둘 이상이고 그 이웃끼리 p 없이도 한 덩어리면 지운다 (끝점 · 줄 가운데는 남음)
 const nb=new Int32Array(16);let chg=true,g=0;while(chg&&g<30){chg=false;g++;for(let y=0;y<h;y++)for(let x=0;x<w;x++){const i=y*w+x;if(!S[i])continue;let m=0;
  for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){if(!dx&&!dy)continue;const X=x+dx,Y=y+dy;if(X>=0&&Y>=0&&X<w&&Y<h&&S[Y*w+X]){nb[2*m]=X;nb[2*m+1]=Y;m++}}
  if(m<2)continue;let seen=1,grow=true;while(grow){grow=false;for(let k=0;k<m;k++){if(seen>>k&1)continue;for(let q=0;q<m;q++)if((seen>>q&1)&&Math.abs(nb[2*k]-nb[2*q])<=1&&Math.abs(nb[2*k+1]-nb[2*q+1])<=1){seen|=1<<k;grow=true;break}}}
  if(seen===(1<<m)-1){S[i]=0;chg=true}}}return S}
function skelGraph(S,w,h){const N8=[-w-1,-w,-w+1,-1,1,w-1,w,w+1],DX=[-1,0,1,-1,1,-1,0,1];
 const nbs=i=>{const x=i%w,o=[];for(let k=0;k<8;k++){const X=x+DX[k];if(X<0||X>=w)continue;const j=i+N8[k];if(j<0||j>=w*h||!S[j])continue;o.push(j)}return o};
 const deg=new Uint8Array(w*h),node=new Int32Array(w*h).fill(-1),vis=new Uint8Array(w*h),pix=[];for(let i=0;i<w*h;i++)if(S[i]){pix.push(i);deg[i]=nbs(i).length}
 const N=[];for(const i of pix){if(deg[i]===2||node[i]>=0)continue;const id=N.length,mem=[i];node[i]=id;
  if(deg[i]>2){const st=[i];while(st.length){const j=st.pop();for(const k of nbs(j))if(deg[k]>2&&node[k]<0){node[k]=id;mem.push(k);st.push(k)}}}
  let sx=0,sy=0;for(const j of mem){sx+=j%w;sy+=(j-j%w)/w}N.push({mem,x:sx/mem.length,y:sy/mem.length,ends:[]})}
 const E=[],pr=new Set(),xy=i=>[i%w,(i-i%w)/w];
 for(let a=0;a<N.length;a++)for(const s of N[a].mem)for(const t of nbs(s)){if(node[t]===a)continue;
  if(node[t]>=0){const b=node[t],k=Math.min(a,b)+'_'+Math.max(a,b)+'_'+Math.min(s,t)+'_'+Math.max(s,t);if(pr.has(k))continue;pr.add(k);
   if(N[a].mem.length>1&&N[b].mem.length>1)continue;E.push({a,b,p:[xy(s),xy(t)]});continue}
  if(vis[t])continue;const path=[xy(s),xy(t)];vis[t]=1;let prev=s,cur=t;
  for(;;){const nx=nbs(cur).filter(k=>k!==prev&&!(node[k]<0&&vis[k]));let k=nx.find(q=>node[q]>=0&&!(node[q]===a&&q===s&&path.length<3));if(k===undefined)k=nx.find(q=>node[q]<0);
   if(k===undefined)break;path.push(xy(k));if(node[k]>=0){cur=k;break}vis[k]=1;prev=cur;cur=k}
  E.push({a,b:node[cur]>=0?node[cur]:-1,p:path})}
 for(const i of pix){if(vis[i]||node[i]>=0)continue;const path=[xy(i)];vis[i]=1;let prev=-1,cur=i;   // 노드 없는 고리
  for(;;){const nx=nbs(cur).filter(k=>k!==prev&&!vis[k]);if(!nx.length)break;vis[nx[0]]=1;path.push(xy(nx[0]));prev=cur;cur=nx[0]}path.push(path[0].slice());E.push({a:-1,b:-1,p:path,loop:true})}
 return {N,E}}
function plenA(P){let L=0;for(let i=1;i<P.length;i++)L+=Math.hypot(P[i][0]-P[i-1][0],P[i][1]-P[i-1][1]);return L}
function graphStrokes(G,B,w,h,hw,spurPx,extend,penR){const {N,E}=G,cutR=hw+1.5,mergeL=Math.max(3,2*hw+2);
 const alive=E.map(()=>true),par=N.map((_,i)=>i),fd=i=>{while(par[i]!==i)i=par[i]=par[par[i]];return i},tips=N.map(()=>[]);
 const nd=(e,s)=>{const v=s?e.b:e.a;return v<0?-1:fd(v)};
 const degs=()=>{const d=new Array(N.length).fill(0);E.forEach((e,i)=>{if(!alive[i]||e.loop)return;const a=nd(e,0),b=nd(e,1);if(a>=0)d[a]++;if(b>=0)d[b]++});return d};
 let pruned=0;for(let r=0;r<4;r++){let any=false;let d=degs();
  E.forEach((e,i)=>{if(!alive[i]||e.loop)return;const a=nd(e,0),b=nd(e,1);if(a<0||b<0)return;if(a===b&&plenA(e.p)<mergeL){alive[i]=false;any=true;return}
   const ta=d[a]===1,tb=d[b]===1;if(((ta&&d[b]>=3)||(tb&&d[a]>=3))&&plenA(e.p)<spurPx){alive[i]=false;pruned++;any=true;d[a]--;d[b]--;const tp=ta?e.p[0]:e.p[e.p.length-1];tips[ta?b:a].push(tp.slice())}});
  d=degs();E.forEach((e,i)=>{if(!alive[i]||e.loop)return;const a=nd(e,0),b=nd(e,1);if(a<0||b<0||a===b)return;if(d[a]>=3&&d[b]>=3&&plenA(e.p)<mergeL){alive[i]=false;any=true;
   const na=N[a],nb=N[b],ma=na.mem.length,mb=nb.mem.length;nb.x=(na.x*ma+nb.x*mb)/(ma+mb);nb.y=(na.y*ma+nb.y*mb)/(ma+mb);nb.mem=nb.mem.concat(na.mem);tips[b]=tips[b].concat(tips[a]);par[a]=b}});
  if(!any)break}
 const d=degs(),Ed=[];E.forEach((e,i)=>{if(alive[i])Ed.push({a:e.loop?-1:nd(e,0),b:e.loop?-1:nd(e,1),p:e.p,loop:!!e.loop})});
 const ends=N.map(()=>[]);Ed.forEach((e,i)=>{if(e.loop)return;if(e.a>=0)ends[e.a].push([i,0]);if(e.b>=0)ends[e.b].push([i,1])});
 const K=(i,s)=>i*2+s,nodeOf=(i,s)=>s?Ed[i].b:Ed[i].a,ctr=n=>[N[n].x,N[n].y],dist=(p,q)=>Math.hypot(p[0]-q[0],p[1]-q[1]);
 const eDir=(e,s)=>{const P=s?e.p.slice().reverse():e.p,L0=Math.max(4,2*hw);let a=0,k=1;for(;k<P.length-1;k++){a+=Math.hypot(P[k][0]-P[k-1][0],P[k][1]-P[k-1][1]);if(a>=L0)break}const q=P[Math.min(k,P.length-1)],dx=q[0]-P[0][0],dy=q[1]-P[0][1],l=Math.hypot(dx,dy)||1;return [dx/l,dy/l]};
 const bent=n=>{const L=[];Ed.forEach(e=>{if(e.loop)return;if(e.a===n)L.push(eDir(e,0));if(e.b===n)L.push(eDir(e,1))});return L.length===2&&L[0][0]*L[1][0]+L[0][1]*L[1][1]>-0.85};   // 두 가지가 148° 보다 꺾임
 const kind=N.map((_,n)=>d[n]>=3?'J':(d[n]===2&&tips[n].length&&bent(n)?'C':(d[n]===1&&N[n].mem.length===1?'T':'P')));     // 갈림 · 모서리 · 끝 · 지남
 const cutOf=n=>(n>=0&&(kind[n]==='J'||kind[n]==='C'))?cutR:0;
 const ori=(i,s)=>s?Ed[i].p.slice().reverse():Ed[i].p.slice();                                           // s 쪽 노드에서 나가는 방향
 const trimmed=(i,s)=>{let P=ori(i,s);const n0=nodeOf(i,s),n1=nodeOf(i,1-s),c0=cutOf(n0),c1=cutOf(n1);
  if(c0>0){const C=ctr(n0);let k=0;while(k<P.length-2&&dist(P[k],C)<c0)k++;P=P.slice(k)}if(c1>0){const C=ctr(n1);let k=P.length-1;while(k>1&&dist(P[k],C)<c1)k--;P=P.slice(0,k+1)}return P};
 const outDir=(P,fromStart)=>{const Q=fromStart?P:P.slice().reverse(),L0=Math.max(3,hw*2);let s=0,k=1;for(;k<Q.length-1;k++){s+=dist(Q[k],Q[k-1]);if(s>=L0)break}
  const a=Q[Math.min(k,Q.length-1)],dx=Q[0][0]-a[0],dy=Q[0][1]-a[1],l=Math.hypot(dx,dy)||1;return [dx/l,dy/l]};          // 끝에서 밖으로
 const link=new Map(),bridge=new Map();
 ends.forEach((L,n)=>{if(kind[n]==='P'&&L.length===2){link.set(K(...L[0]),L[1]);link.set(K(...L[1]),L[0]);return}
  if(kind[n]==='C'&&L.length===2){const tp=tips[n].reduce((a,q)=>[a[0]+q[0]/tips[n].length,a[1]+q[1]/tips[n].length],[0,0]);link.set(K(...L[0]),L[1]);link.set(K(...L[1]),L[0]);bridge.set(K(...L[0]),[tp]);bridge.set(K(...L[1]),[tp]);return}
  if(kind[n]!=='J')return;const V=L.map(([i,s])=>{const P=trimmed(i,s);return outDir(P,true).map(v=>-v)}),free=L.map(()=>true);   // 노드에서 가지로 나가는 방향
  for(;;){let best=0.34,bi=-1,bj=-1;for(let a=0;a<L.length;a++)for(let b=a+1;b<L.length;b++){if(!free[a]||!free[b])continue;const c=-(V[a][0]*V[b][0]+V[a][1]*V[b][1]);if(c>best){best=c;bi=a;bj=b}}
   if(bi<0)break;free[bi]=free[bj]=false;link.set(K(...L[bi]),L[bj]);link.set(K(...L[bj]),L[bi]);const mid=best>0.94?[]:[ctr(n)];bridge.set(K(...L[bi]),mid);bridge.set(K(...L[bj]),mid)}});
 // 끝 처리: 갈림길에서 끝나는 가지는 같은 노드를 지나는 다리까지 곧게, 없으면 노드 가운데 · 줄 끝은 검은 곳 끝 − 펜 반지름까지
 const segsAt=n=>{const S=[];ends[n].forEach(([i,s])=>{const o=link.get(K(i,s));if(!o)return;const A=trimmed(i,s)[0],Bp=trimmed(o[0],o[1])[0],m=bridge.get(K(i,s))||[];const Q=[A,...m,Bp];for(let k=1;k<Q.length;k++)S.push([Q[k-1],Q[k]])});return S};
 const rayHit=(p,dv,S,maxT)=>{let best=null;for(const [a,b] of S){const ex=b[0]-a[0],ey=b[1]-a[1],den=dv[0]*ey-dv[1]*ex;if(Math.abs(den)<1e-9)continue;
   const t=((a[0]-p[0])*ey-(a[1]-p[1])*ex)/den,u=((a[0]-p[0])*dv[1]-(a[1]-p[1])*dv[0])/den;if(t>=-0.5&&t<=maxT&&u>=-0.05&&u<=1.05&&(!best||t<best[2]))best=[p[0]+t*dv[0],p[1]+t*dv[1],t]}return best};
 const march=(p,dv)=>{let t=0;for(;t<200;t+=0.5){const X=Math.round(p[0]+dv[0]*t),Y=Math.round(p[1]+dv[1]*t);if(X<0||Y<0||X>=w||Y>=h||!B[Y*w+X])break}return t};
 const endPt=(P,atStart,n)=>{if(n<0)return null;const dv=outDir(P,atStart),e=atStart?P[0]:P[P.length-1];
  if(kind[n]==='J'){const hit=rayHit(e,dv,segsAt(n),4*cutR);return hit?[hit[0],hit[1]]:ctr(n)}
  if(kind[n]==='T'&&extend){const ext=march(e,dv)-0.5-penR;return ext>0.3?[e[0]+dv[0]*ext,e[1]+dv[1]*ext]:null}return null};
 const used=new Uint8Array(Ed.length),out=[];
 const A=p=>[p[0],p[1],1];                                                                                 // 매끈하게 할 때 움직이지 않는 점(모서리 · 다리 끝)
 const walk=(i,s)=>{let P=trimmed(i,s),ci=i,cs=s;const first=[i,s];for(;;){used[ci]=1;const nx=link.get(K(ci,1-cs));if(!nx||used[nx[0]]&&!(nx[0]===first[0]&&nx[1]===first[1]))return {P,closed:false,last:[ci,1-cs]};
   const m=(bridge.get(K(ci,1-cs))||[]).map(A),pl=P.length-1,hasB=cutOf(nodeOf(ci,1-cs))>0;if(hasB)P[pl]=A(P[pl]);
   if(nx[0]===first[0]&&nx[1]===first[1]){if(hasB)P[0]=A(P[0]);P=P.concat(m,[P[0].slice()]);return {P,closed:true}}
   const Q=trimmed(nx[0],nx[1]);if(hasB)Q[0]=A(Q[0]);P=P.concat(m,Q);ci=nx[0];cs=nx[1]}};
 Ed.forEach((e,i)=>{if(used[i]||e.loop)return;for(const s of [0,1])if(!link.has(K(i,s))){const o=walk(i,s);const a=endPt(o.P,true,nodeOf(i,s)),b=endPt(o.P,false,nodeOf(...o.last));
   if(a)o.P.unshift(a);if(b)o.P.push(b);out.push(o.P);break}});
 Ed.forEach((e,i)=>{if(used[i])return;if(e.loop){used[i]=1;out.push(e.p)}else out.push(walk(i,0).P)});
 const had=new Uint8Array(N.length);E.forEach(e=>{if(e.a>=0)had[e.a]=1;if(e.b>=0)had[e.b]=1});N.forEach((nn,n)=>{if(had[n]||fd(n)!==n||nn.mem.length>64)return;out.push([[nn.x,nn.y]])});        // 점(골격이 한 점) — 한 번 찍기
 return {S:out,pruned}}
function smoothP(P,sig){if(sig<=0||P.length<4)return P;const n=P.length,closed=Math.hypot(P[0][0]-P[n-1][0],P[0][1]-P[n-1][1])<1e-6,R=Math.ceil(2*sig),W=[];
 for(let k=-R;k<=R;k++)W.push(Math.exp(-k*k/(2*sig*sig)));const anc=P.map((p,i)=>!!p[2]||(!closed&&(i===0||i===n-1)));
 if(closed&&!anc.some(Boolean)){const m=n-1,out=[];for(let i=0;i<m;i++){let sx=0,sy=0,sw=0;for(let k=-R;k<=R;k++){const j=((i+k)%m+m)%m,wt=W[k+R];sx+=P[j][0]*wt;sy+=P[j][1]*wt;sw+=wt}out.push([sx/sw,sy/sw])}out.push(out[0].slice());return out}
 if(closed)anc[0]=anc[n-1]=true;const out=P.map(p=>p.slice());
 for(let i=0;i<n;i++){if(anc[i])continue;let a=i,b=i;while(a>0&&!anc[a])a--;while(b<n-1&&!anc[b])b++;const lim=Math.min(i-a,b-i);let sx=0,sy=0,sw=0;
  for(let k=-Math.min(R,lim);k<=Math.min(R,lim);k++){const wt=W[k+R];sx+=P[i+k][0]*wt;sy+=P[i+k][1]*wt;sw+=wt}out[i]=[sx/sw,sy/sw]}return out}
function sharpen(P,hw){const n=P.length;if(n<5)return P;const closed=Math.hypot(P[0][0]-P[n-1][0],P[0][1]-P[n-1][1])<1e-6,S=[0];
 for(let i=1;i<n;i++)S.push(S[i-1]+Math.hypot(P[i][0]-P[i-1][0],P[i][1]-P[i-1][1]));const T=S[n-1],L1=Math.max(3,1.5*hw),La=Math.max(2,hw+1),Lb=La+Math.max(6,3*hw);
 const at=s=>{if(closed)s=((s%T)+T)%T;else s=Math.max(0,Math.min(T,s));let k=1;while(k<n-1&&S[k]<s)k++;const t=(s-S[k-1])/Math.max(1e-9,S[k]-S[k-1]);return [P[k-1][0]+t*(P[k][0]-P[k-1][0]),P[k-1][1]+t*(P[k][1]-P[k-1][1])]};
 const turn=(i,L)=>{if(!closed&&(S[i]<L||T-S[i]<L))return 0;const a=at(S[i]-L),b=at(S[i]+L),u=[P[i][0]-a[0],P[i][1]-a[1]],v=[b[0]-P[i][0],b[1]-P[i][1]],nu=Math.hypot(...u),nv=Math.hypot(...v);
  if(nu<1e-6||nv<1e-6)return 0;return Math.acos(Math.max(-1,Math.min(1,(u[0]*v[0]+u[1]*v[1])/(nu*nv))))};
 const th=P.map((_,i)=>turn(i,L1)),cand=[];for(let i=1;i<n-1;i++){if(th[i]<0.6)continue;let j=i,mx=th[i];while(j+1<n-1&&th[j+1]>=0.6){j++;mx=Math.max(mx,th[j])}     // 꺾임 구간 [i, j]
  if(mx>=0.7){const sm=(S[i]+S[j])/2;let c=i;while(c<j&&S[c]<sm)c++;const La2=Math.max(La,(S[j]-S[i])/2+2),Lb2=La2+Math.max(6,3*hw);if(turn(c,Lb2)>0.5)cand.push([c,La2,Lb2])}i=j}
 if(!cand.length)return P;const rep=[];
 for(const [i,La,Lb] of cand){const a0=at(S[i]-Lb),a1=at(S[i]-La),b0=at(S[i]+La),b1=at(S[i]+Lb),d1=[a1[0]-a0[0],a1[1]-a0[1]],d2=[b1[0]-b0[0],b1[1]-b0[1]],den=d1[0]*d2[1]-d1[1]*d2[0];
  if(Math.abs(den)<1e-9)continue;const res=(p0,p1,s0,s1)=>{const dx=p1[0]-p0[0],dy=p1[1]-p0[1],l=Math.hypot(dx,dy)||1;let m=0;for(let k=0;k<n;k++){let sk=S[k];if(closed){const dd=((sk-S[i])%T+1.5*T)%T-T/2;sk=S[i]+dd}if(sk<s0||sk>s1)continue;m=Math.max(m,Math.abs(dy*(P[k][0]-p0[0])-dx*(P[k][1]-p0[1]))/l)}return m};
  const tolR=Math.max(0.6,0.05*(Lb-La));if(res(a0,a1,S[i]-Lb,S[i]-La)>tolR||res(b0,b1,S[i]+La,S[i]+Lb)>tolR)continue;                                   // 다리가 곧아야 모서리
  const t=((b0[0]-a1[0])*d2[1]-(b0[1]-a1[1])*d2[0])/den,X=[a1[0]+t*d1[0],a1[1]+t*d1[1]];if(t<-0.2||Math.hypot(X[0]-P[i][0],X[1]-P[i][1])>2*hw+4)continue;
  rep.push({i,s0:S[i]-La,s1:S[i]+La,X})}
 if(!rep.length)return P;const out=[];let r=0;for(let i=0;i<n;i++){const c=rep[r];if(c&&S[i]>c.s0&&S[i]<c.s1){if(S[i]>=S[c.i]){out.push([c.X[0],c.X[1],1]);r++;while(i+1<n&&S[i+1]<c.s1)i++}continue}out.push(P[i])}
 return out}
function rdpA(P,eps){const idx=[0];for(let i=1;i<P.length-1;i++)if(P[i][2])idx.push(i);idx.push(P.length-1);let out=[];
 for(let k=1;k<idx.length;k++){const seg=rdp(P.slice(idx[k-1],idx[k]+1).map(p=>[p[0],p[1]]),eps);out=out.length?out.concat(seg.slice(1)):seg}return out}
function orderStrokes(S,start,join){const n=S.length,t0=Date.now();if(!n)return {S,tr0:0,tr1:0,lifts:0};
 const d=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]),cl=P=>P.length>2&&d(P[0],P[P.length-1])<1e-6;
 let tr0=d(start,S[0][0]);for(let i=1;i<n;i++)tr0+=d(S[i-1][S[i-1].length-1],S[i][0]);
 const left=new Set(S.map((_,i)=>i)),seq=[];let cur=start;
 while(left.size){let bi=-1,bd=1e18,bm=0;for(const i of left){const P=S[i];if(cl(P)){for(let k=0;k<P.length-1;k+=Math.max(1,Math.floor(P.length/64))){const v=d(cur,P[k]);if(v<bd){bd=v;bi=i;bm=2+k}}}
   else{const a=d(cur,P[0]),b=d(cur,P[P.length-1]);if(a<bd){bd=a;bi=i;bm=0}if(b<bd){bd=b;bi=i;bm=1}}}
  let P=S[bi];if(bm===1)P=P.slice().reverse();else if(bm>=2){const k=bm-2;P=P.slice(k,P.length-1).concat(P.slice(0,k+1))}
  seq.push(P);left.delete(bi);cur=P[P.length-1]}
 const st=i=>seq[i][0],en=i=>seq[i][seq[i].length-1];let imp=true,pass=0;
 while(imp&&pass<20&&Date.now()-t0<600&&seq.length<2500){imp=false;pass++;for(let i=0;i<seq.length-1;i++){const pe=i?en(i-1):start;
   for(let j=i+1;j<seq.length;j++){const nx=j+1<seq.length?st(j+1):null,dl=d(pe,en(j))+(nx?d(st(i),nx):0)-d(pe,st(i))-(nx?d(en(j),nx):0);
    if(dl<-1e-9){const r=seq.slice(i,j+1).reverse().map(P=>P.slice().reverse());seq.splice(i,j-i+1,...r);imp=true}}}}
 const out=[seq[0].slice()];for(let i=1;i<seq.length;i++){const L=out[out.length-1];if(d(L[L.length-1],seq[i][0])<=join)out[out.length-1]=L.concat(seq[i].slice(d(L[L.length-1],seq[i][0])<1e-9?1:0));else out.push(seq[i].slice())}
 let tr1=d(start,out[0][0]);for(let i=1;i<out.length;i++)tr1+=d(out[i-1][out[i-1].length-1],out[i][0]);return {S:out,tr0,tr1,lifts:out.length-1}}
function major3(B,w,h){const O=new Uint8Array(w*h);for(let y=0;y<h;y++)for(let x=0;x<w;x++){let c=0;for(let dy=-1;dy<=1;dy++){const Y=y+dy;if(Y<0||Y>=h)continue;for(let dx=-1;dx<=1;dx++){const X=x+dx;if(X>=0&&X<w)c+=B[Y*w+X]}}O[y*w+x]=c>=5?1:0}return O}
function keepBlobs(B,S,D,w,h){const lab=new Int32Array(w*h).fill(-1),st=[];let nAdd=0;          // 얇게 깎다 통째로 없어진 덩어리 → 가장 굵은 곳에 한 점
 for(let i0=0;i0<w*h;i0++){if(!B[i0]||lab[i0]>=0)continue;lab[i0]=i0;st.push(i0);let has=0,bi=i0;
  while(st.length){const i=st.pop(),x=i%w;if(S[i])has=1;if(D[i]>D[bi])bi=i;for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const X=x+dx,j=i+dy*w+dx;if(X<0||X>=w||j<0||j>=w*h||!B[j]||lab[j]>=0)continue;lab[j]=i0;st.push(j)}}
  if(!has){S[bi]=1;nAdd++}}return nAdd}
function centerTrace(B0,w,h,o){let B=B0,D=distT(B,w,h),S=thinZS(B,w,h);keepBlobs(B,S,D,w,h);const med=()=>{const v=[];for(let i=0;i<w*h;i++)if(S[i]&&D[i]<1e8)v.push(D[i]-0.5);v.sort((a,b)=>a-b);return v.length?v[v.length>>1]:0};
 if(med()>=1.5){B=major3(B0,w,h);D=distT(B,w,h);S=thinZS(B,w,h);keepBlobs(B,S,D,w,h)}                                                 // 굵은 선: 가장자리 한 화소 혹 · 홈을 다수결로 지우고 다시
 const G=skelGraph(S,w,h),hwMed=med(),spur=Math.max(o.spurPx,2*hwMed+1),R=graphStrokes(G,B,w,h,hwMed,spur,o.extend,o.penRpx);
 const P=R.S.map(p=>p.length<2?[[p[0][0],p[0][1]]]:rdpA(smoothP(sharpen(p,hwMed),o.sig),o.tol)).filter(p=>p.length>=1);let nFill=0;
 for(let j=0;j<P.length;j++){const Q=P[j];if(Q.length<2)continue;const n=Q.length,L=plenA(Q);if(Math.hypot(Q[0][0]-Q[n-1][0],Q[0][1]-Q[n-1][1])<1e-6)continue;   // 짧고 굵은 덩어리(골격이 짧은 토막) → 점
  let a=0,c=Q[0];for(let k=1;k<n;k++){const l=Math.hypot(Q[k][0]-Q[k-1][0],Q[k][1]-Q[k-1][1]);if(a+l>=L/2){const t=(L/2-a)/Math.max(l,1e-9);c=[Q[k-1][0]+t*(Q[k][0]-Q[k-1][0]),Q[k-1][1]+t*(Q[k][1]-Q[k-1][1])];break}a+=l}
  const xi=Math.round(c[0]),yi=Math.round(c[1]);let r=0;for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const X=xi+dx,Y=yi+dy;if(X>=0&&Y>=0&&X<w&&Y<h&&D[Y*w+X]<1e8)r=Math.max(r,D[Y*w+X]-0.5)}
  if(L<=2*r+1)P[j]=[c]}
 if(o.fillDots&&o.penRpx>0)for(let j=0;j<P.length;j++){if(P[j].length!==1)continue;const Q=dotSpiral(P[j][0],D,w,h,o.penRpx);if(Q){P[j]=Q;nFill++}}   // 굵은 점: 펜 굵기로 안쪽에서 바깥으로 나선
 return {S:P,hwMed,pruned:R.pruned,nodes:G.N.length,edges:G.E.length,nFill}}
function dotSpiral(c,D,w,h,pr){const xi=Math.round(c[0]),yi=Math.round(c[1]);let r=0;
 for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const X=xi+dx,Y=yi+dy;if(X>=0&&Y>=0&&X<w&&Y<h&&D[Y*w+X]<1e8)r=Math.max(r,D[Y*w+X]-0.5)}
 const rr=r-pr;if(rr<0.75*pr)return null;const pitch=1.6*pr,Q=[[c[0],c[1]]],TW=2*Math.PI;let th=0,end=-1;      // 나선 간격 = 펜 굵기의 80 % (겹치게) · 끝에 한 바퀴 더
 for(let it=0;it<20000;it++){const rad=Math.min(rr,pitch*th/TW);if(rad>=rr&&end<0)end=th+TW;if(end>=0&&th>=end)break;th+=Math.min(0.5,1/Math.max(rad,1));const rd=Math.min(rr,pitch*th/TW);Q.push([c[0]+rd*Math.cos(th),c[1]+rd*Math.sin(th)])}
 return Q}
