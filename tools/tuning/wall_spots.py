import numpy as np, yaml, sys, os
from scipy import ndimage
m=yaml.safe_load(open("devkit_ws/src/racer_mapping/maps/iros2026/track_solid.yaml")); raw=open("devkit_ws/src/racer_mapping/maps/iros2026/track_solid.pgm","rb").read()
p=raw.split(maxsplit=4); w,h=int(p[1]),int(p[2]); img=np.frombuffer(p[4][:w*h],np.uint8).reshape(h,w); res=m["resolution"]; ox,oy=m["origin"][:2]
D=ndimage.distance_transform_edt(img>=250)*res
dist=lambda x,y: D[int(h-1-(y-oy)/res), int((x-ox)/res)]
spots={"C2exit":(3.0,4.3,-10.5,-8.0),"chevronlane":(2.3,3.4,-4.0,-1.8),"s40bend":(3.2,4.3,1.6,2.8)}
for f in sys.argv[1:]:
    L=np.loadtxt(f,delimiter=",",comments="#")
    out=[]
    for nm,(x0,x1,y0,y1) in spots.items():
        d=[dist(r[1],r[2]) for r in L if x0<=r[1]<=x1 and y0<=r[2]<=y1]
        out.append("%s %.2f" % (nm, min(d)) if d else nm+" --")
    print("%-60s %s | kmax %.2f | %.2f m" % (os.path.basename(f), " | ".join(out), np.abs(L[:,4]).max(), L[-1,0]))
