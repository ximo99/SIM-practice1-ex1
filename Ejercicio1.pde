// Authors:  //<>//
// Julián Rodríguez
// Ximo Casanova

// Problem description:
// El plano inclinado con doble muelle

// Differential equations:
// F = m * a
// C1 = L * cos(angulo)
// C2 = L * sin(angulo)
// Fn = m * g * cos(angulo)
// Ffa = v * - Kd
// Fr = v * - u

// Definitions:
enum IntegratorType
{
  NONE,
  EXPLICIT_EULER, 
  SIMPLECTIC_EULER, 
  HEUN, 
  RK2, 
  RK4
}

// Parameters of the numerical integration:
final boolean REAL_TIME = true;
float SIM_STEP = 0.01;  // Simulation time-step (s)
IntegratorType _integrator = IntegratorType.EXPLICIT_EULER;  // ODE integration method

// Display values:
final boolean FULL_SCREEN = false;
final int DRAW_FREQ = 50;  // Draw frequency (Hz or Frame-per-second)
int DISPLAY_SIZE_X = 1000; // Display width (pixels)
int DISPLAY_SIZE_Y = 1000; // Display height (pixels)

// Draw values:
final int [] BACKGROUND_COLOR = {200, 200, 255};
final int [] REFERENCE_COLOR = {0, 255, 0};
final int [] OBJECTS_COLOR = {255, 0, 0};
final float OBJECTS_SIZE = 1.0;   // Size of the objects (m)
final float PIXELS_PER_METER = 20.0;   // Display length that corresponds with 1 meter (pixels)
final PVector DISPLAY_CENTER = new PVector(0.0, 0.0);   // World position that corresponds with the center of the display (m)

// Parameters of the problem:
final int L = 20; // Longitud de la superficie
final float M = 1.0;  // Particle mass (kg)
final float Kr = 1.2;  // Constante de rozamiento del plano
final float Kd = 1.3;  // Constante de rozamiento del aire
final float Ke1 = 5.0; // Constante elástica del muelle 1
final float Ke2 = 5.0; // Constante elástica del muelle 2
final float E1 = 1.0;  // Elongación en reposo del muelle 1
final float E2 = 1.0;  // Elongación en reposo del muelle 2
final float angle = 45.0; // Angulo (en grados)
final float ang = radians(angle); // Conversión a radianes del angulo en grados

PVector c0 = new PVector(0.0, 0.0); // Punto C0
PVector c1 = new PVector(L*cos(ang), 0.0); // Punto del muelle C1
PVector c2 = new PVector(0.0, L*sin(ang)); // Punto del muelle C2
PVector L1 = new PVector(-cos(radians(90)-ang)*L, sin(radians(90)-ang)*L);  // Punto del plano L1
PVector L2 = new PVector(-cos(radians(90)-ang)*L, sin(radians(90)-ang)*L);  // Punto del plano L2

PVector plano = PVector.sub(c2,c1);

final float Gc = 9.801;   // Gravity constant (m/(s*s))
final PVector G = new PVector(0.0, -Gc);   // Acceleration due to gravity (m/(s*s))

boolean plane = true;

// Variables to be solved:
PVector s = new PVector(0.0, 0.0);    // Position of the particle (m)
PVector v = new PVector(0.0, 0.0);    // Velocity of the particle (m/s)
PVector a = new PVector(0.0, 0.0);    // Accleration of the particle (m/(s*s))

// Time control:
int _lastTimeDraw = 0;  // Last measure of time in draw() function (ms)
float _simTime = 0.0;  // Simulated time (s)
float _deltaTimeDraw = 0.0;  // Time between draw() calls (s)
float _elapsedTime = 0.0;  // Elapsed (real) time (s)

// Output control:
//PrintWriter _output;
//final String FILE_NAME = "data.txt";

// Auxiliary variables:
float _energy;   // Total energy of the particle (J)

// Main code:

// Converts distances from world length to pixel length
float worldToPixels(float dist)
{
  return dist*PIXELS_PER_METER;
}

// Converts distances from pixel length to world length
float pixelsToWorld(float dist)
{
  return dist/PIXELS_PER_METER;
}

// Converts a point from world coordinates to screen coordinates
void worldToScreen(PVector worldPos, PVector screenPos)
{
  screenPos.x = 0.5*DISPLAY_SIZE_X + (worldPos.x - DISPLAY_CENTER.x)*PIXELS_PER_METER;
  screenPos.y = 0.5*DISPLAY_SIZE_Y - (worldPos.y - DISPLAY_CENTER.y)*PIXELS_PER_METER;
}

// Converts a point from screen coordinates to world coordinates
void screenToWorld(PVector screenPos, PVector worldPos)
{
  worldPos.x = ((screenPos.x - 0.5*DISPLAY_SIZE_X)/PIXELS_PER_METER) + DISPLAY_CENTER.x;
  worldPos.y = ((0.5*DISPLAY_SIZE_Y - screenPos.y)/PIXELS_PER_METER) + DISPLAY_CENTER.y;
}

void drawStaticEnvironment()
{
  background(BACKGROUND_COLOR[0], BACKGROUND_COLOR[1], BACKGROUND_COLOR[2]);

  textSize(20);
  text("Sim. Step = " + SIM_STEP + " (Real Time = " + REAL_TIME + ")", width*0.025, height*0.075);  
  text("Integrator = " + _integrator, width*0.025, height*0.1);
  text("Energy = " + _energy + " J", width*0.025, height*0.125);
  text("Ángulo del plano = " + angle + "º", width*0.025, height*0.150);
  text("Plano activado = " + plane, width*0.025, height*0.175);
  
  fill(REFERENCE_COLOR[0], REFERENCE_COLOR[1], REFERENCE_COLOR[2]);
  strokeWeight(1);
  
  PVector screenPos = new PVector();
  worldToScreen(new PVector(), screenPos);
  
  // Triángulo
  // Linia de c2 a c0
  line(screenPos.x, screenPos.y, worldToPixels(c2.x) + screenPos.x, - worldToPixels(c2.y) + screenPos.y);
  
  // Linia de c2 a c1
  if (plane)
    line(worldToPixels(c2.x) + screenPos.x, - worldToPixels(c2.y) + screenPos.y, worldToPixels(c1.x) + screenPos.x, screenPos.y);
  
  // Linia de c0 a c1
  line(worldToPixels(c0.x) + screenPos.x, - worldToPixels(c0.y) + screenPos.y, worldToPixels(c1.x) + screenPos.x, screenPos.y);
  
  // Muelle C1
  fill(255);
  worldToScreen(c1, screenPos);
  circle(screenPos.x, screenPos.y, worldToPixels(OBJECTS_SIZE));
  
  // Muelle C2
  fill(255);
  worldToScreen(c2, screenPos);
  circle(screenPos.x, screenPos.y, worldToPixels(OBJECTS_SIZE));
}

void drawMovingElements()
{
  fill(OBJECTS_COLOR[0], OBJECTS_COLOR[1], OBJECTS_COLOR[2]);
  strokeWeight(2);
  
  PVector screenPos = new PVector();
  worldToScreen(s, screenPos);
  
  // Masa
  square(screenPos.x + s.x, screenPos.y + s.y, worldToPixels(OBJECTS_SIZE));
  
  // Muelles
  screenPos = new PVector();
  PVector p1 = new PVector();
  PVector p2 = new PVector();

  
  worldToScreen(s, screenPos);
  worldToScreen(c1, p1);
  worldToScreen(c2, p2);
  
  line(p1.x, p1.y, screenPos.x, screenPos.y);
  line(p2.x, p2.y, screenPos.x, screenPos.y);
}

void PrintInfo()
{
  /*println("Energy: " + _energy + " J");
  println("Elapsed time = " + _elapsedTime + " s");
  println("Simulated time = " + _simTime + " s \n");*/
    
  /*_output.println("Simulated time = " + _simTime + " s");
  _output.println("Simulation time-step = " + SIM_STEP + " s");
  _output.println("Position = " + s + " m");
  _output.println("Speed = " + v + " m/s");
  _output.println("Energy = " + _energy + " J");
  _output.println("\n -------------------------------------------- \n");*/
}

void initSimulation()
{
  //_output = createWriter("data.txt");
  _simTime = 0.0;
  _elapsedTime = 0.0;
  
  s.set(L*cos(ang)/2, L*sin(ang)/2);
  v.set(0.0, 0.0);
  a.set(0.0, 0.0);
  
  v.set(v);
}

void updateSimulation()
{
  switch (_integrator)
  {
    case EXPLICIT_EULER:
      updateSimulationExplicitEuler();
    break;
  
    case SIMPLECTIC_EULER:
      updateSimulationSimplecticEuler();
    break;
  
    case HEUN:
      updateSimulationHeun();
    break;
  
    case RK2:
      updateSimulationRK2();
    break;
  
    case RK4:
      updateSimulationRK4();
    break;
  }
  
  _simTime += SIM_STEP;
}

void updateSimulationExplicitEuler()
{
  // Calcular la derivada en el principio del intervalo
  a = calculateAcceleration(s, v);
  
  // Calcular la posición siguiente a partir de la velocidad en el principio del intervalo
  s.add(PVector.mult(v, SIM_STEP));
  
  // Calcular la velocidad siguiente a partir de la derivada en el principio del intervalo
  v.add(PVector.mult(a, SIM_STEP));
}

void updateSimulationSimplecticEuler()
{
  // Calcular la derivada en el principio del intervalo
  a = calculateAcceleration(s, v);
  
  // Calcular la velocidad siguiente a partir de la derivada en el principio del intervalo  
  v.add(PVector.mult(a, SIM_STEP));
  
  // Calcular la posición siguiente a partir de la velocidad en el principio del intervalo  
  s.add(PVector.mult(v, SIM_STEP));  
}

void updateSimulationHeun()
{
  // Parte 1, integración numérica de la velocidad
  // Calcular aceleracion a
  a = calculateAcceleration(s, v);
  
  // Paso de Euler, actualizo s2 y v2 (velocidad y posición al final del intervalo)
  PVector s2 = new PVector();
  s2 = s;
  s2.add(PVector.mult(v, SIM_STEP));
  PVector v2 = new PVector();
  v2 = v;
  
  // Cálculo de la velocidad promedio a partir de v y v2
  PVector v_prom = PVector.mult(PVector.add(v, v2), 0.5);
  
  //actualizar s con la v promedio
  s.add(PVector.mult(v_prom, SIM_STEP));
  
  // Parte 2, integración de la acceleración
  // Calcular la aceleración a2 (aceleración al final del intervalo)
  PVector a2 = new PVector();
  a2 = calculateAcceleration(s2, v2);
  
  // Cálculo de la aceleración promedio a partir de a y a2
  PVector a_prom = PVector.mult(PVector.add(a, a2), 0.5);
  
  //Actualizar la velocidad con la aceleración promedio
  v.add(PVector.mult(a_prom, SIM_STEP));
}

void updateSimulationRK2()
{
  //Calcular acceleracion a
  a = calculateAcceleration(s,v);
  
  // k1s = v(t) * h
  PVector k1s = PVector.mult(v, SIM_STEP);
  
  // k1v = a(s(t), v(t)) * h
  PVector k1v = PVector.mult(a, SIM_STEP);
  
  PVector s2 = PVector.add(s, PVector.mult(k1s, 0.5));
  PVector v2 = PVector.add(v, PVector.mult(k1v, 0.5));
  PVector a2 = calculateAcceleration(s2,v2);
  
  // k2v = a(s(t) + k1s / 2, v(t) + k1v / 2) * h
  PVector k2v = PVector.mult(a2,SIM_STEP);
  
  // k2s = (v(t)+k1v/2)*h
  PVector k2s = PVector.mult(PVector.add(v,PVector.mult(k1v,0.5)),SIM_STEP);
  
  v.add(k2v);
  s.add(k2s);
}

void updateSimulationRK4()
{
  // Calcular acceleracion a
  a = calculateAcceleration(s,v);

  // k1v = a(s(t), v(t)) * h
  PVector k1v = PVector.mult(a,SIM_STEP);
  
  // k1s = v(t) * h
  PVector k1s = PVector.mult(v, SIM_STEP);
  
  // k2v = a(s(t) + k1s / 2, v(t) + k1v / 2) * h  
  PVector s2 = PVector.add(s, PVector.mult(k1s, 0.5));
  PVector v2 = PVector.add(v, PVector.mult(k1v, 0.5));
  PVector a2 = calculateAcceleration(s2,v2);
  PVector k2v = PVector.mult(a2,SIM_STEP);
  PVector k2s = PVector.mult(PVector.add(v,PVector.mult(k1v,0.5)),SIM_STEP);
  
  PVector s3 = PVector.add(s, PVector.mult(k2s, 0.5));
  PVector v3 = PVector.add(v, PVector.mult(k2v, 0.5));
  PVector a3 = calculateAcceleration(s3,v3);
  
  // k3v = a(s(t)+k2s/2, v(t)+k2v/2)*h
  PVector k3v = PVector.mult(a3,SIM_STEP);
  
  // k3s = (v(t)+k2v/2)*h
  PVector k3s = PVector.mult(PVector.add(v,PVector.mult(k2v,0.5)),SIM_STEP);
  
  PVector s4 = PVector.add(s, k3s);
  PVector v4 = PVector.add(v, k3v);
  PVector a4 = calculateAcceleration(s4,v4);
  
  // k4v = a(s(t)+k3s, v(t)+k3v)*h
  PVector k4v = PVector.mult(a4,SIM_STEP);
  
  // k4s = (v(t)+k3v)*h
  PVector k4s = PVector.mult(PVector.add(v,k3s),SIM_STEP);
  
  // v(t+h) = v(t) + (1/6)*k1v + (1/3)*k2v + (1/3)*k3v +(1/6)*k4v
  v.add(PVector.mult(k1v,1/6.0));
  v.add(PVector.mult(k2v, 1/3.0));
  v.add(PVector.mult(k3v, 1/3.0));
  v.add(PVector.mult(k4v, 1/6.0));
  
  // s(t+h) = s(t) + (1/6)*k1s + (1/3)*k2s + (1/3)*k3s +(1/6)*k4s
  s.add(PVector.mult(k1s,1/6.0));  
  s.add(PVector.mult(k2s, 1/3.0));
  s.add(PVector.mult(k3s, 1/3.0));
  s.add(PVector.mult(k4s, 1/6.0));
}

PVector calculateAcceleration(PVector s, PVector v)
{  
  PVector Fm1;  // Fuerza del muelle 1
  PVector Fm2;  // Fuerza del muelle 2
  PVector Fm;   // Fuerza total de los muelles
  
  PVector Frd;  // Fuerza de rozamiento del aire
  PVector Frp;  // Fuerza de rozamiento del plano
  PVector Fr;   // Fuerza de rozamiento total
  
  PVector D, N;  // Dirección del plano y la normal
  PVector Fn = new PVector();    // Fuerza normal
  PVector Fp = new PVector();    // Fuerza peso
  
  PVector F;     // Fuerza total
  PVector a;     // Acceleración
  
  PVector L1 = new PVector();  // Punto donde está el muelle L1
  PVector L2 = new PVector();  // Punto donde está el muelle L2
  
  L1 = c1;
  L2 = c2;
  
  Fp = PVector.mult(G, M);
  
  D = (new PVector(cos(radians(90)-ang), -sin(radians(90)-ang))).normalize();
  //N = (new PVector(sin(radians(90)-ang), cos(radians(90)-ang))).normalize();
  N = new PVector (plano.y, -plano.x);
  Fn = PVector.mult(N.normalize(), Fp.y*cos(ang));
  
  Frd = PVector.mult(v, -Kd);
  Frp = PVector.mult(v.copy().normalize(), - PVector.dist(Fn,new PVector(0,0))* Kr);
  
  if (plane)
    Fr = PVector.add(Frd, Frp);
  else
    Fr = Frd;
  
  Fm1 = PVector.mult(PVector.sub(L1, s), Ke1);
  Fm2 = PVector.mult(PVector.sub(L2, s), Ke2);
  Fm = PVector.add(Fm1, Fm2);
  
  if(plane)
    F = PVector.add(PVector.mult(D, Fp.y), Fr);
  else
    F = PVector.add(Fp, Fr);
    
  F.add(Fm);
  
  a = PVector.div(F, M);
  
  return a;
}

void calculateEnergy()
{  
  float Ec, Ep, El; // Energías (cinética, potencial y elástica)
  float El1, El2;   // Energía elástica del muelle 1 y del muelle 2
  
  Ec = 0.5 * M * sq(v.mag());
  Ep = M * Gc * s.y;
  
  float diff_long = sqrt(sq(s.x - c1.x) + sq(s.y - c1.y));
  El1 = 0.5 * Ke1 * sq(diff_long - E1);
  
  diff_long = sqrt(sq(s.x - c2.x) + sq(s.y - c2.y));
  El2 = 0.5 * Ke2 * sq(diff_long - E2);
  
  El = El1 + El2;
  _energy = Ec + Ep + El;
}

void settings()
{
  if (FULL_SCREEN)
  {
    fullScreen();
    DISPLAY_SIZE_X = displayWidth;
    DISPLAY_SIZE_Y = displayHeight;
  } 
  else
    size(DISPLAY_SIZE_X, DISPLAY_SIZE_Y);
}

void setup()
{
  frameRate(DRAW_FREQ);
  _lastTimeDraw = millis();

  initSimulation();
}

void draw()
{
  int now = millis();
  _deltaTimeDraw = (now - _lastTimeDraw)/1000.0;
  _elapsedTime += _deltaTimeDraw;
  _lastTimeDraw = now;

  //println("\nDraw step = " + _deltaTimeDraw + " s - " + 1.0/_deltaTimeDraw + " Hz");
  
  if (REAL_TIME)
  {
    float expectedSimulatedTime = 1.0*_deltaTimeDraw;
    float expectedIterations = expectedSimulatedTime/SIM_STEP;
    int iterations = 0;
    
    for (; iterations < floor(expectedIterations); iterations++)
      updateSimulation();
    
    if ((expectedIterations - iterations) > random(0.0, 1.0))
    {
      updateSimulation();
      iterations++;
    }
    
    //println("Expected Simulated Time: " + expectedSimulatedTime);
    //println("Expected Iterations: " + expectedIterations);
    //println("Iterations: " + iterations);
  } 
  else
    updateSimulation();

  drawStaticEnvironment();
  drawMovingElements();

  calculateEnergy();
  PrintInfo();
}

void mouseClicked() 
{
  PVector mouse = new PVector(mouseX,mouseY);
  PVector new_pos = new PVector();
  
  screenToWorld(mouse, new_pos);
  s.set(new_pos.x, new_pos.y);
  v = new PVector();
  a = new PVector();
  
  _simTime = 0.0;
  _elapsedTime = 0.0;
}

void keyPressed()
{
  switch(key)
  {
    case 'e':
    case 'E':
      _integrator = IntegratorType.EXPLICIT_EULER;
    break;
    
    case 's':
    case 'S':
      _integrator = IntegratorType.SIMPLECTIC_EULER;
    break;
    
    case 'h':
    case 'H':
      _integrator = IntegratorType.HEUN;
    break;
    
    case '2':
      _integrator = IntegratorType.RK2;
    break;
    
    case '4':
      _integrator = IntegratorType.RK4;
    break;
    
    case 'r':
    case 'R':
      initSimulation();
    break;
    
    case '+':
      SIM_STEP += SIM_STEP;
    break;
    
    case '-':
      SIM_STEP -= SIM_STEP;
    break;
    
    case 'p':
    case 'P':
        plane = !plane;
        initSimulation();
    break;
    
    case 'q':
    case 'Q':
      stop();
    break;
  }
}

void stop()
{
  //_output.flush();
  //_output.close();
  exit();
}
