#!/usr/bin/env python
# -*- coding: utf-8 -*-
from matplotlib import pyplot
from mpl_toolkits import mplot3d
import struct
import numpy



def slice(self,S,display=True,direction=[0,0,1]):
    '''
    
    '''
    slice_lines=[]
    [x_max_new,x_min_new,y_max_new,y_min_new,z_max_new,z_min_new,rot_mat]=self.rotated_max_min(direction)
    print(numpy.floor((z_max_new-z_min_new)))
    print('Slicing in the direction [{0[0]:.3f},{0[1]:.3f},{0[2]:.3f}]; Total layers: {1:.0f}'.format(direction,numpy.floor((z_max_new-z_min_new)/S)+1))

    for i in range(self.triangle_numbers):
        X_1=numpy.array([self.triangles[i][0][0],self.triangles[i][0][1],self.triangles[i][0][2]])
        X_2=numpy.array([self.triangles[i][1][0],self.triangles[i][1][1],self.triangles[i][1][2]])
        X_3=numpy.array([self.triangles[i][2][0],self.triangles[i][2][1],self.triangles[i][2][2]])

        X_1=numpy.dot(rot_mat,X_1)
        X_2=numpy.dot(rot_mat,X_2)
        X_3=numpy.dot(rot_mat,X_3)

        [x1,y1,z1]=X_1
        [x2,y2,z2]=X_2
        [x3,y3,z3]=X_3

        [x1,x2,x3]=[x1-x_min_new,x2-x_min_new,x3-x_min_new]
        [y1,y2,y3]=[y1-y_min_new,y2-y_min_new,y3-y_min_new]
        [z1,z2,z3]=[z1-z_min_new,z2-z_min_new,z3-z_min_new]

        
        vert_tup=[(x1,y1,z1),(x2,y2,z2),(x3,y3,z3)] #三角形/3个顶点

        # 如果斜角在切片平面上
        if(z1==z2 and z2==z3):
            continue

        L1=(z1/S) #确定点下方或点上最近的切片平面的步骤
        L2=(z2/S)
        L3=(z3/S)
        
        
        L=numpy.floor([L1,L2,L3])
        
        on_layer_flag=0
        odd=0
        
        #确定平行于切片平面的边
        if (z1==z2):
            on_layer_flag+=1
            odd=3
            
        if (z2==z3):
            on_layer_flag+=1
            odd=1
        if (z1==z3):
            on_layer_flag+=1
            odd=2
        
        L_max=numpy.floor(max(L1,L2,L3))
        L_min=numpy.floor(max(L1,L2,L3))

        
        # 如果一条边平行于切片平面
        if(on_layer_flag==1): 
            zs=[1,2,3]
            zs.remove(odd)

            #如果切片平面上的平行边
            if(vert_tup[zs[0]-1][2]%S==0):
                line_vertices=[vert_tup[zs[0]-1],vert_tup[zs[1]-1]] #平行边
                slice_lines.append(line_vertices)
                
                line_vertices=[]

                #如果有更多的切片平面与三角形相交
                if((L[odd-1]>L[zs[0]-1]+1 or L[odd-1]<L[zs[0]-1]-1)):
                    [xa,ya,za]=[vert_tup[odd-1][0],vert_tup[odd-1][1],vert_tup[odd-1][2]]
                    [xb,yb,zb]=[vert_tup[zs[0]-1][0],vert_tup[zs[0]-1][1],vert_tup[zs[0]-1][2]]
                    [xc,yc,zc]=[vert_tup[zs[1]-1][0],vert_tup[zs[1]-1][1],vert_tup[zs[1]-1][2]]
                    
                    
                    z_p1=L[odd-1]*S
                    if(za<zb): z_p1=z_p1+S
                    
                    while abs(z_p1-zb)>0:
                        if za==z_p1:
                            z_p1=z_p1+(S*(zb-z_p1))/abs(zb-z_p1)
                            continue
                        
                        x_p1=(xa-xb)*(z_p1-zb)/(za-zb)+xb
                        y_p1=(ya-yb)*(z_p1-zb)/(za-zb)+yb
                        
                        x_p2=(xa-xc)*(z_p1-zc)/(za-zc)+xc
                        y_p2=(ya-yc)*(z_p1-zc)/(za-zc)+yc
                        
                        line_vertices=[(x_p1,y_p1,z_p1),(x_p2,y_p2,z_p1)]
                        
                        slice_lines.append(line_vertices)
                        if zb>z_p1:
                            z_p1=z_p1+S
                        else:
                            z_p1=z_p1-S
                        
            #如果平行边不在切片平面上       
            elif ((L[odd-1]>L[zs[0]-1] or L[odd-1]<L[zs[0]-1])):
                [xa,ya,za]=[vert_tup[odd-1][0],vert_tup[odd-1][1],vert_tup[odd-1][2]]
                [xb,yb,zb]=[vert_tup[zs[0]-1][0],vert_tup[zs[0]-1][1],vert_tup[zs[0]-1][2]]
                [xc,yc,zc]=[vert_tup[zs[1]-1][0],vert_tup[zs[1]-1][1],vert_tup[zs[1]-1][2]]
                z_p1=(L[odd-1])*S
                if(za<zb): z_p1=z_p1+S
                              
                while (z_p1-zb)/(za-zb)>=0:
                    if za==z_p1:
                        z_p1=z_p1+(S*(zb-z_p1))/abs(zb-z_p1)
                        continue
                    
                    x_p1=(xa-xb)*(z_p1-zb)/(za-zb)+xb
                    y_p1=(ya-yb)*(z_p1-zb)/(za-zb)+yb
                    
                    x_p2=(xa-xc)*(z_p1-zc)/(za-zc)+xc
                    y_p2=(ya-yc)*(z_p1-zc)/(za-zc)+yc
                    
                    line_vertices=[(x_p1,y_p1,z_p1),(x_p2,y_p2,z_p1)]
                    
                    slice_lines.append(line_vertices)
                    if zb>z_p1:
                        z_p1=z_p1+S
                    else:
                        z_p1=z_p1-S

        #剩下的三角形           
        else:
            vert_tup.sort(key=lambda x:x[2])
            [xa,ya,za]=[vert_tup[0][0],vert_tup[0][1],vert_tup[0][2]]
            [xb,yb,zb]=[vert_tup[1][0],vert_tup[1][1],vert_tup[1][2]]
            [xc,yc,zc]=[vert_tup[2][0],vert_tup[2][1],vert_tup[2][2]]

          
            L1=numpy.floor(za/S)
            L2=numpy.floor(zb/S)
            L3=numpy.floor(zc/S)

            if (L1<L3):
                z_p1=L3*S

                #从最高顶点开始向下移动
                while(z_p1>=zb):
                    if(z_p1==zc):
                        z_p1=z_p1-S
                        continue
                    
                    x_p1=(xc-xb)*(z_p1-zb)/(zc-zb)+xb
                    y_p1=(yc-yb)*(z_p1-zb)/(zc-zb)+yb

                    x_p2=(xc-xa)*(z_p1-za)/(zc-za)+xa
                    y_p2=(yc-ya)*(z_p1-za)/(zc-za)+ya

                    line_vertices=[(x_p1,y_p1,z_p1),(x_p2,y_p2,z_p1)]
                    slice_lines.append(line_vertices)
                    z_p1=z_p1-S


                while(z_p1>za):
                    
                    x_p1=(xa-xb)*(z_p1-zb)/(za-zb)+xb
                    y_p1=(ya-yb)*(z_p1-zb)/(za-zb)+yb

                    x_p2=(xa-xc)*(z_p1-zc)/(za-zc)+xc
                    y_p2=(ya-yc)*(z_p1-zc)/(za-zc)+yc

                    line_vertices=[(x_p1,y_p1,z_p1),(x_p2,y_p2,z_p1)]
                    slice_lines.append(line_vertices)
                    z_p1=z_p1-S
    
            
            
      

    # le=len(slice_lines)

    # [x_min,x_max]=[0,x_max_new-x_min_new]
    # [y_min,y_max]=[0,y_max_new-y_min_new]
    # [z_min,z_max]=[0,z_max_new-z_min_new]

    # if (display):
    #     figure=pyplot.figure()
    #     ax=figure.add_subplot(111,projection='3d')
    #     for v in range(le):
    #         ax.plot([slice_lines[v][0][0],slice_lines[v][1][0]],[slice_lines[v][0][1],slice_lines[v][1][1]],[slice_lines[v][0][2],slice_lines[v][1][2]])

    #     x_range=x_max
    #     y_range=y_max
    #     z_range=z_max

    #     max_range=max(x_range,y_range,z_range)
    #     half_range=max_range/2.0

    #     x_mean=0.5*(x_max)
    #     y_mean=0.5*(y_max)
    #     z_mean=0.5*(z_max)

    #     ax.auto_scale_xyz([x_mean-half_range,x_mean+half_range],[y_mean-half_range,y_mean+half_range],[z_mean-half_range,z_mean+half_range])
    #     ax.set_aspect('equal', adjustable='box')

    #     pyplot.axis('off')
    #     pyplot.show()

    return slice_lines
