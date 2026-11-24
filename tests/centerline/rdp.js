function rdp(P,eps){if(P.length<3)return P;const keep=new Uint8Array(P.length);keep[0]=keep[P.length-1]=1;const st=[[0,P.length-1]];
 while(st.length){const [a,b]=st.pop();const ax=P[a][0],ay=P[a][1],dx=P[b][0]-ax,dy=P[b][1]-ay,L=Math.hypot(dx,dy);let dm=0,im=-1;
  for(let i=a+1;i<b;i++){const d=L>1e-9?Math.abs(dy*(P[i][0]-ax)-dx*(P[i][1]-ay))/L:Math.hypot(P[i][0]-ax,P[i][1]-ay);if(d>dm){dm=d;im=i}}
  if(dm>eps&&im>0){keep[im]=1;st.push([a,im],[im,b])}}return P.filter((_,i)=>keep[i])}
