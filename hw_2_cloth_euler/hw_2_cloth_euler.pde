SpringSystem ss;

int loose = 10;
PVector SphereCenter = new PVector(600,350,200); float radius=110;
float ks=2000; float kd =10;
float windforce = 0;
int windtime=0;

void mouseClicked() {
  windforce=5;
  windtime=10;
}

void setup() {
  size(1200,800,P3D);
  ss = new SpringSystem(300,300,450,100,0);
  //frameRate(20);
  //frameRate(30);
}

// set up camera;
int x=1200, y=50, z=500;
void ChangeCamera() {
  if (keyPressed) {
    if (key == CODED) {
      if (keyCode == UP) {
         y=y+1;     
      }
      if (keyCode == DOWN) {
        y=y-1;
      }
      if (keyCode == LEFT) {
        x=x+1;
        z=z-1;
      }
      if (keyCode == RIGHT) {
        x=x-1;
        z=z+1;
      }
    }
  }
} 

void draw() {
  background(0);
  camera(x,y,z,600,450,0,0.0,1.0,0.0);
  for (int i=0; i<10;i++) {
    ss.update(0.01);
  }
  ss.display();
  print(frameRate);
  println();
}


class SpringSystem {
  PVector[][] velocity;
  PVector[][] position;
  //PVector[][] acceleration;
  
  SpringSystem(int len, int wid, int originX, int originY,int originZ) {
    int m= (int) len/loose;
    int n= (int) wid/loose;
    velocity= new PVector[m][n];
    position= new PVector[m][n];
    //acceleration= new PVector[m][n];
    
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
}

  //uodate velocity and position    
  void update(float dt) {
    ChangeCamera();
    
    PVector[][] vn = velocity;
    
    //for horizontal strings
    for (int i=0;i<velocity.length-1;i++) {
      for (int j=0;j<velocity[0].length;j++) {
        PVector e = PVector.sub(position[i+1][j],position[i][j]);
        float l = e.mag();
        e = PVector.div(e,l);
        float v1 = PVector.dot(e,velocity[i][j]);
        float v2 = PVector.dot(e,velocity[i+1][j]);
        float f = -ks*(loose-l) -kd*(v1-v2);
        vn[i][j] = PVector.add(vn[i][j],PVector.mult(e,f*dt));
        vn[i+1][j] = PVector.sub(vn[i+1][j],PVector.mult(e,f*dt));
      }
    }
    
    //for vertical strings
    for (int i=0;i<velocity.length;i++) {
      for (int j=0;j<velocity[0].length-1;j++) {
        PVector e = PVector.sub(position[i][j+1],position[i][j]);
        float l = e.mag();
        e = PVector.div(e,l);
        float v1 = PVector.dot(e,velocity[i][j]);
        float v2 = PVector.dot(e,velocity[i][j+1]);
        float f = -ks*(loose-l) -kd*(v1-v2);
        vn[i][j] = PVector.add(vn[i][j],PVector.mult(e,f*dt));
        vn[i][j+1] = PVector.sub(vn[i][j+1],PVector.mult(e,f*dt));
      }
    }
    
    
    // add gravity force
    for (int i=0;i<velocity.length;i++) {
      for (int j=0;j<velocity[0].length;j++) {
        PVector gravity = new PVector(0,1,0);
        vn[i][j]=PVector.add(vn[i][j],gravity);
      }
    }
    
     // add wind force
     for (int i=0;i<velocity.length;i++) {
      for (int j=velocity[0].length/2;j<velocity[0].length;j++) {
        PVector wind = new PVector(0,0,-windforce);
        vn[i][j]=PVector.add(vn[i][j],wind);
      }
    }
    
    //update wind force time and end wind force
    if (windtime>0) {
      windtime--;
    }
    
    if (windtime==0) {
      windforce=0;
    }
    
    
    // restrict up line for cloth
    for (int i=0;i<velocity.length;i++) {
      vn[i][0]=new PVector(0,0,0);
    }
      
    // update position
    for (int i=0;i<velocity.length;i++) {
     for (int j=0;j<velocity[0].length;j++) {
       position[i][j]=PVector.add(position[i][j],PVector.mult(vn[i][j],dt));
      }
    }
    
    // update velocity
    velocity=vn;
    
    // check collision
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