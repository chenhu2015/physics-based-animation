SpringSystem ss;

int loose = 10;
PVector SphereCenter = new PVector(600,350,200); float radius=110;
float ks=2000; float kd =8;

void setup() {
  size(1200,800,P3D);
  ss = new SpringSystem(300,400,450,100,0);
  camera(1200,50,500,600,450,0,0.0,1.0,0.0);
}

void draw() {
  background(0);
  for (int i=0; i<10;i++) {
    ss.update(0.01);
  }
  //ss.display();
  //print(frameRate);
  //println();
}

class SpringSystem {
  PVector[][] velocity;
  PVector[][] position;
  //PVector[][] acceleration;
  PVector[][] force;
  
  SpringSystem(int len, int wid, int originX, int originY,int originZ) {
    int m= (int) len/loose;
    int n= (int) wid/loose;
    velocity= new PVector[m][n];
    position= new PVector[m][n];
    //acceleration= new PVector[m][n];
    force = new PVector[m][n];
    
    for (int i=0;i<m;i++) {
      for (int j=0;j<n;j++) {
        PVector p = new PVector (originX+loose*i,originY,originZ+loose*j);
        position[i][j]=p;
      }
    }
    
    for (int i=0;i<m;i++) {
      for (int j=0;j<n;j++) {
        velocity[i][j]= new PVector(0,0,0);
    }
  }
  //  for (int j=0;j<n;j++) {
   //   velocity[m-1][j]=new PVector(j*100,0);
   // }
}


    
  void update(float dt) {  
    PVector[][] vn = velocity;
    for (int i=0;i<velocity.length-1;i++) {
      for (int j=0;j<velocity[0].length;j++) {
        PVector e = PVector.sub(position[i+1][j],position[i][j]);
        float l = e.mag();
        e = PVector.div(e,l);
        float v1 = PVector.dot(e,velocity[i][j]);
        float v2 = PVector.dot(e,velocity[i+1][j]);
        float f = -ks*(10-l) -kd*(v1-v2);
        vn[i][j] = PVector.add(vn[i][j],PVector.mult(e,f*dt));
        vn[i+1][j] = PVector.sub(vn[i+1][j],PVector.mult(e,f*dt));
      }
    }

    for (int i=0;i<velocity.length;i++) {
      for (int j=0;j<velocity[0].length-1;j++) {
        PVector e = PVector.sub(position[i][j+1],position[i][j]);
        float l = e.mag();
        e = PVector.div(e,l);
        float v1 = PVector.dot(e,velocity[i][j]);
        float v2 = PVector.dot(e,velocity[i][j+1]);
        float f = -ks*(8-l) -kd*(v1-v2);
        vn[i][j] = PVector.add(vn[i][j],PVector.mult(e,f*dt));
        vn[i][j+1] = PVector.sub(vn[i][j+1],PVector.mult(e,f*dt));
      }
    }
    
    // zero out force each time update
    for (int i=0;i<position.length;i++) {
      for (int j=0;j<position[0].length;j++) {
        force[i][j]=new PVector(0.0,0.0,0.0);
      }
    }
    
    //// NEED ERROR CHECKING!!!!!!
     // calculate drag force
    float rocd = 0.001;
    for (int i=0;i<position.length-1;i++) {
      for (int j=0;j<position[0].length-1;j++) {
        PVector v1_ave = PVector.div( (PVector.add(PVector.add(vn[i][j],vn[i+1][j]),vn[i][j+1])), 3.0);
        float v1_magn = v1_ave.mag();
        PVector boundary1 = PVector.sub(position[i+1][j],position[i][j]);
        PVector boundary2 = PVector.sub(position[i][j+1],position[i][j]);
        PVector cross_product = boundary1.cross(boundary2);
        float magn = cross_product.mag();
        //print (magn +" ");
        PVector sum = PVector.mult(cross_product, (PVector.dot(v1_ave,cross_product))*v1_magn/(2.0*magn));
        //print (sum+ " ");
        PVector f_drag1 = PVector.mult(sum,-0.5*rocd);
        print(f_drag1+ " ");
        PVector.add(force[i][j],PVector.div(f_drag1,3.0));
        PVector.add(force[i+1][j],PVector.div(f_drag1,3.0));
        PVector.add(force[i][j+1],PVector.div(f_drag1,3.0));

                     
        PVector v2_ave=PVector.add(PVector.add(vn[i+1][j],vn[i][j+1]),vn[i+1][j+1]);
        float v2_magn = v2_ave.mag();
        PVector boundary3 = PVector.sub(position[i][j+1],position[i+1][j]);
        PVector boundary4 = PVector.sub(position[i+1][j+1],position[i+1][j]);
        PVector cross_product2 = boundary3.cross(boundary4);
        float magn2 = cross_product2.mag();
        PVector n2 = PVector.div(cross_product2,magn2);
        float a_zero2 = magn2/2.0;
        float a2 =(PVector.dot(v2_ave,n2))*a_zero2/v2_magn;
        PVector f_drag2 = PVector.mult(n2,-0.5*rocd*v2_magn*v2_magn*a2);
        PVector.add(force[i+1][j],PVector.div(f_drag2,3));
        PVector.add(force[i][j+1],PVector.div(f_drag2,3));
        PVector.add(force[i+1][j+1],PVector.div(f_drag2,3));
      }
    }
       
    for (int i=0;i<velocity.length;i++) {
      for (int j=0;j<velocity[0].length;j++) {
        PVector gravity = new PVector(0,1,0);
        force[i][j].add(gravity);
        //print(force[i][j] +" ");   
        vn[i][j]=PVector.add(vn[i][j],force[i][j]);
      }
    }
    
    for (int i=0;i<velocity.length;i++) {
      vn[i][0]=new PVector(0,0,0); 
    }
    
    for (int i=0;i<velocity.length;i++) {
     for (int j=0;j<velocity[0].length;j++) {
       position[i][j]=PVector.add(position[i][j],PVector.mult(vn[i][j],dt));
      }
    }
      
    velocity=vn;
      
    for (int i=0;i<velocity.length;i++) {
      for (int j=0;j<velocity[0].length;j++) {
        float d= PVector.dist(position[i][j],SphereCenter);
        if (d<radius+0.5) {
          PVector n = PVector.sub(position[i][j],SphereCenter);
          n.normalize();
          float v1 = PVector.dot(velocity[i][j],n);
          PVector bounce = PVector.mult(n,v1);
          velocity[i][j].sub(PVector.mult(bounce,1.0));
          position[i][j].add(PVector.mult(n,1.2+radius-d));
      } 
    }
  }  
}
  
  void display() {
    noStroke();
    fill(0,255,10);
    lights();
    translate(SphereCenter.x,SphereCenter.y,SphereCenter.z);
    sphere(radius);
    translate(-SphereCenter.x,-SphereCenter.y,-SphereCenter.z);
    
    for (int i=0;i<position.length-1;i++) {
      for (int j=0;j<position[0].length-1;j++) {
        
        int colour = j%2;
        if (colour ==0) {
        fill(10,10,255);
        stroke(10,10,255);
        }
        else {
        fill(255,0,0);
        stroke(255,0,0);
        }
        beginShape();
        vertex(position[i][j].x,position[i][j].y,position[i][j].z);
        vertex(position[i+1][j].x,position[i+1][j].y,position[i+1][j].z);
        vertex(position[i][j+1].x,position[i][j+1].y,position[i][j+1].z);        
        endShape();
        
        beginShape();
        vertex(position[i+1][j].x,position[i+1][j].y,position[i+1][j].z);
        vertex(position[i][j+1].x,position[i][j+1].y,position[i][j+1].z); 
        vertex(position[i+1][j+1].x,position[i+1][j+1].y,position[i+1][j+1].z);
        endShape();
      }
    }
  }
}