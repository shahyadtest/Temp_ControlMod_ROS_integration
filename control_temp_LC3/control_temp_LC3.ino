uint32_t LoopTimer;
const int PWM_PIN = 3;     // PWM output to the actuator
const int LED_PIN = 9;     // Status LED
float Temp; //Temperatura medida
float Temp_1=0.0;
float a=0.9391; // parametro filtro IIR
float ref=30.0;  //Referencia temperatura
float u=0.0,u_1=0.0;//Se_al de control
float e=0.0,e_1=0.0;// error resp
float K0=18.004,K1=-17.9773; //parametros control PI discreto
float tiempo=0;
static bool ref_step_done = false;

float ref_cmd = 0.0;
bool ref_override = false;

uint32_t last_ref_cmd_ms = 0;
const uint32_t REF_TIMEOUT_MS = 10000; // 10 s


void read_uart_commands() {
  static char buf[32];
  static uint8_t idx = 0;

  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\n' || c == '\r') {
      buf[idx] = '\0';
      idx = 0;

      // Formato esperado: "REF:35.0"  o "REF:OFF"
      if (strncmp(buf, "REF:", 4) == 0) {

        if (strcmp(buf, "REF:OFF") == 0) {
          ref_override = false;
        } else {
          float v = atof(buf + 4);

          // Seguridad: limita rango (ajusta si quieres)
          if (v < 0.0) v = 0.0;
          if (v > 120.0) v = 120.0;

          ref_cmd = v;
          ref_override = true;
          last_ref_cmd_ms = millis(); // watchdog (último comando recibido)
        }
      }

    } else {
      if (idx < sizeof(buf) - 1) buf[idx++] = c;
    }
  }
}





void setup() {
  // ===== TEENSY SERIAL =====
  // USB (GUI / debug)
  // Serial.begin(115200);      // <-- enable for GUI over USB
  // UART hardware (Raspberry Pi)
  Serial1.begin(57600);       // <-- enable for UART to RP4B (TX1/RX1 + GND)
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  ref=(float(analogRead(A8))*3.3/1023.0-0.5)/0.01;
  LoopTimer=micros();
}


void loop() {
  read_uart_commands();
  // Watchdog: si no llegan comandos en 10 s, volver a ref interna
  if (ref_override && (millis() - last_ref_cmd_ms > REF_TIMEOUT_MS)) {
    ref_override = false;
  }
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  Temp = (1.0-a)*(float(analogRead(A8))*3.3/1023.0-0.5)/0.01+a*Temp_1; // fitro digital medicion temperatura
  Temp_1=Temp;
  if (ref_override) {
    ref = ref_cmd;   // referencia desde ROS
  } else {
    // referencia interna (tu lógica actual)
    if (!ref_step_done && tiempo >= 180.0) {
      ref = ref + 10.0;
      ref_step_done = true;
    }
  }
  e=(ref-Temp); //cierre de lazo de control  

  // Control PI
  u = u_1 + K0*e + K1*e_1; //controlador PI discreto
  if (u >= 100.0){      //saturacion
    u = 100.0;
  } 
  
  if (u <= 0.0 || ref==0.0){
    u = 0.0;
  }
  e_1=e;
  u_1=u;
  analogWrite(PWM_PIN,map(u,0,100,0,255));
 // UART hardware (Raspberry Pi)
  Serial1.print(Temp);
  Serial1.print(",");
  Serial1.print(u);
  Serial1.print(",");
  Serial1.println(ref);
  while ( micros() - LoopTimer< 100000); // el codigo se bloquea o no avanza hasta que se cumpla la condicion
  LoopTimer=micros();
  tiempo=tiempo+0.1;
}