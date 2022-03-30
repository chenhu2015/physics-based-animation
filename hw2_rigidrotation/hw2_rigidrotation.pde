RigidBodySystem bs;

void setup() {
  size(1200,800,P3D);
  bs = new RigidBodySystem(); 
}

//update rate is 10 times than display rate
void draw() {
  background(0);
  for (int i =0; i<10;i++) {
  bs.update(0.1);
  }
  bs.display();
}

//set rigid body parameters
float len = 400; float hei = 10; float initialX =400; float initialY=50;

//set obstacle parameters
float [] obs = {500,200,250,550}; 

//begin for rigid body system
class RigidBodySystem {
  float mass =1;
  float I;
  float [] Gravity={0,9.8};
  
  float [] position = new float [2];
  float angle=0;
  float[] velocity ={0,0};
  float rotationvelocity  = 0;
  float [] Force = {0,0};
  float[] ForcePosition = {0,0};
  float torque; 
  float[] Momentum={0,0};
  float AngMomentum=0;
  
  float[] BodyBoundary = new float [8];
  int time=0;
  int DetectPosition=0;
  
  RigidBodySystem() {
    position[0]=initialX;
    position[1]=initialY;
    I = 1.0/12.0*mass*(len*len+hei*hei);
}  
  
  // decide whether the rigid body hits obstacle
  boolean DetectHit() {
    
    // calculate the boundary of my rigid body
    // this can be updated for more abstract boundary conditions
    float c = cos(angle);
    float s = sin(angle);
    BodyBoundary[0] = c*len/2 - s*hei/2 + position[0];
    BodyBoundary[1] = s*len/2 + c*hei/2 + position[1];
    
    BodyBoundary[2] = -c*len/2 - s*hei/2 + position[0];
    BodyBoundary[3] = -s*len/2 + c*hei/2 + position [1];
    
    BodyBoundary[4] = -c*len/2 + s*hei/2 + position[0];
    BodyBoundary[5] = -s*len/2 - c*hei/2 + position[1];
    
    BodyBoundary[6] = c*len/2 + s*hei/2 + position[0];
    BodyBoundary[7] = s*len/2 - c*hei/2 + position[1];   
    
    // calculate boundary vectors
    float [] ver = new float [8];
    ver[0] = BodyBoundary[0] - BodyBoundary[2];
    ver[1] = BodyBoundary[1] - BodyBoundary[3];
    PVector vector1 = new PVector (ver[0],ver[1]);
    
    ver[2] = BodyBoundary[2] - BodyBoundary[4];
    ver[3] = BodyBoundary[3] - BodyBoundary[5];
    PVector vector2 = new PVector (ver[2],ver[3]);
    
    ver[4] = BodyBoundary[4] - BodyBoundary[6];
    ver[5] = BodyBoundary[5] - BodyBoundary[7];
    PVector vector3 = new PVector (ver[4],ver[5]);
    
    ver[6] = BodyBoundary[6] - BodyBoundary[0];
    ver[7] = BodyBoundary[7] - BodyBoundary[1];
    PVector vector4 = new PVector (ver[6],ver[7]);
    
    // calculate obstacle/boundary vector
    // this can be updated for more abstract boundary conditions and more abstract obstacle condition
    for (DetectPosition=0;DetectPosition<obs.length;DetectPosition=DetectPosition+2) {
    float [] new_ver = new float [8];
    new_ver[0] = BodyBoundary[0] - obs[DetectPosition];
    new_ver[1] = BodyBoundary[1] - obs[DetectPosition+1];
    PVector new_vector1 = new PVector (new_ver[0],new_ver[1]);
    
    new_ver[2] = BodyBoundary[2] - obs[DetectPosition];
    new_ver[3] = BodyBoundary[3] - obs[DetectPosition+1];
    PVector new_vector2 = new PVector (new_ver[2],new_ver[3]);
    
    new_ver[4] = BodyBoundary[4] - obs[DetectPosition];
    new_ver[5] = BodyBoundary[5] - obs[DetectPosition+1];
    PVector new_vector3 = new PVector (new_ver[4],new_ver[5]);
    
    new_ver[6] = BodyBoundary[6] - obs[DetectPosition];
    new_ver[7] = BodyBoundary[7] - obs[DetectPosition+1];
    PVector new_vector4 = new PVector (new_ver[6],new_ver[7]); 
    
       
    // calculate cross products of two vectors   
    float a = (vector1.cross(new_vector1)).z;
    float b = (vector2.cross(new_vector2)).z;
    float cc = (vector3.cross(new_vector3)).z;
    float d = (vector4.cross(new_vector4)).z;
    
    // if all cross products are position, obstacles is inside rigid body
    if ( (a>0) && (b>0) && (cc>0) && (d>0) ){
      return true;   
      }
    }
    return false;
   } 
  
  // calculate torque applied
  void ApplyForce() {
    PVector a = new PVector (Force[0],Force[1],0);
    PVector b = new PVector (ForcePosition[0]-position[0],ForcePosition[1]-position[1],0);
    PVector c = a.cross(b);
    if (c.z >0) {
      torque = c.mag();
    }
    else {
      torque = -c.mag();
    }
  }
  
  
  // update position and rotation angle
  void update(float dt) {
    ApplyForce();
    
    //update position
    float AllForceX = Force[0]+Gravity[0];
    float AllForceY = Force[1]+Gravity[1];
    Momentum[0] = AllForceX*dt;
    Momentum[1] = AllForceY*dt;
    position[0] = position[0] + Momentum[0]/mass *dt;
    position[1] = position[1] + Momentum[1]/mass *dt;
    
    //update rotation angle
    AngMomentum = AngMomentum+torque *dt;
    rotationvelocity = AngMomentum/I;
    angle = angle+ rotationvelocity*dt;
    while (angle>2*PI) {
      angle =angle - 2*PI;
    }
    
    // check if rigid body hits obstacle. If hits, then update force and force position 
    if (DetectHit() == true && time == 0 ) {
    //applied force value should related to velocity, but I use constant here. This can be revised in the future.
    //applied force dirtection is related with rotationvelocity sign.
      if (rotationvelocity >=0) {
        float xx = - (obs[DetectPosition+1] - position[1]);
        float yy = obs[DetectPosition] - position[0];
        float mag = sqrt(xx*xx+yy*yy);
        Force[0] = xx/mag*50;
        Force[1] = yy/mag*50;
        ForcePosition[0] = obs[DetectPosition];
        ForcePosition[1] = obs[DetectPosition+1];
        // time is used to waive disturb that once it, in a small time of period, there shouldn't be another hit again.
        time =8;
      }
      else {
        float xx = obs[DetectPosition+1] - position[1];
        float yy = -(obs[DetectPosition] - position[0]);
        float mag = sqrt(xx*xx+yy*yy);
        Force[0] = xx/mag*50;
        Force[1] = yy/mag*50;
        ForcePosition[0] = obs[DetectPosition];
        ForcePosition[1] = obs[DetectPosition+1];
        time =8;    
      }
    }
      
      // if not hit, waive the force and force position set up
      else {
        Force[0] = 0;
        Force[1] = 0;
        ForcePosition[0] = 0;
        ForcePosition[1] = 0;
      }
      
      // decrease hit time range
      if (time >0) {
        time--;
      }
  }
  
  void display() { 
        background(255,255,255); 
        fill(255,0,0);
        for (int i=0;i<obs.length;i=i+2) {
        ellipse(obs[i],obs[i+1],8,8);
        }
        fill(0,200,10);
        noStroke();
        pushMatrix();
        translate(position[0],position[1]);
        rotate(angle);
        rect(-len/2.0,-hei/2.0,len,hei);
        popMatrix();
      }
}