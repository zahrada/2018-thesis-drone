// Arduino Měřič vzdálenosti ultrazvukový

// piny pro připojení Trig a Echo z modulu
int pTrig = 4;
int pEcho = 5;
// inicializace proměnných, do kterých se uloží data
long odezva, vzdalenost;
 
void setup() {
  // nastavení pinů modulu jako výstup a vstup
  pinMode(pTrig, OUTPUT);
  pinMode(pEcho, INPUT);
  // komunikace přes sériovou linku rychlostí 9600 baud
  Serial.begin(9600);
}

void loop()
{
  // nastavíme na 2 mikrosekundy výstup na GND (pro jistotu)
  // poté nastavíme na 5 mikrosekund výstup rovný napájení
  // a poté opět na GND
  digitalWrite(pTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(pTrig, LOW);
  // pomocí funkce pulseIn získáme následně
  // délku pulzu v mikrosekundách (us)
  odezva = pulseIn(pEcho, HIGH);
  // přepočet získaného času na vzdálenost v cm
  vzdalenost = odezva / 58.31;
  Serial.print("Vzdalenost je ");
  Serial.print(vzdalenost);
  Serial.println(" cm.");
  // pauza 0.5 s pro přehledné čtení
  delay(500);
}

// http://navody.arduino-shop.cz/navody-k-produktum/meric-vzdalenosti-ultrazvukovy.html
