#define F_CPU 16000000
#define RSPIN PD0
#define ENPIN PD1

/////////////////////////////
/// 		LCD FUNCTIONS 		///
/////////////////////////////

void lcd_init(void)
{
  wait_ms(15);      // Wait before LCD activation
  lcd_action(0x02); // Setare mod 4 biti
  lcd_action(0x28); // Initializare pe 2 linii
  lcd_action(0x0c); // Dezactivare Cursor
  lcd_action(0x06); // Incrementare Cursor
  lcd_action(0x01); // Clean LCD
  wait_us(200);
}

void lcd_action(unsigned char cmnd)
{
  PORTD = (PORTD & 0x0F) | (cmnd & 0xF0);
  PORTD &= ~(1 << RSPIN);
  PORTD |= (1 << ENPIN);
  wait_us(1);
  PORTD &= ~(1 << ENPIN);
  wait_us(200);
  PORTD = (PORTD & 0x0F) | (cmnd << 4);
  PORTD |= (1 << ENPIN);
  wait_us(1);
  PORTD &= ~(1 << ENPIN);
  wait_us(200);
}

void lcd_print(char *str)
{
  for (int i = 0; str[i] != 0; i++)
  {
    PORTD = (PORTD & 0x0F) | (str[i] & 0xF0);
    PORTD |= (1 << RSPIN);
    PORTD |= (1 << ENPIN);
    wait_us(1);
    PORTD &= ~(1 << ENPIN);
    wait_us(200);
    PORTD = (PORTD & 0x0F) | (str[i] << 4);
    PORTD |= (1 << ENPIN);
    wait_us(1);
    PORTD &= ~(1 << ENPIN);
    wait_ms(2);
  }
}

void lcd_print_pos(char row, char pos, char *str)
{
  if (row == 0 && pos < 16)
    lcd_action((pos & 0x0F) | 0x80);
  else if (row == 1 && pos < 16)
    lcd_action((pos & 0x0F) | 0xC0);
  lcd_print(str);
}

/////////////////////////////////
/// 		LCD FUNCTIONS END 		///
/////////////////////////////////

///////////////////////////////////////
/// 		DISTANCE SENSOR FUNCTION 		///
///////////////////////////////////////

long read_ultrasonic_distance(int triggerPin, int echoPin)
{

  PORTB &= ~(1 << triggerPin); // Clear the trigger (set it low)
  wait_us(2);
  PORTB |= (1 << triggerPin); // Set triggerPin high
  wait_us(10);
  PORTB &= ~(1 << triggerPin); // Clear the trigger again

  unsigned long startTime = micros();
  while (!(PINB & (1 << echoPin)))
    if (micros() - startTime > 19000) // Timeout if no response within 19ms
      return 1000000;
  startTime = micros();

  unsigned long endTime = micros();
  while (PINB & (1 << echoPin))
    endTime = micros();

  return endTime - startTime;
}

///////////////////////////////////////////
/// 		DISTANCE SENSOR FUNCTION END 		///
///////////////////////////////////////////

///////////////////////////////
/// 		TIMER FUNCTIONS 		///
///////////////////////////////

void wait_us(int wait_time)
{
  int cycles = (int)wait_time * (F_CPU / 1024) / 1000000;
  TCNT1 = 0;
  while (TCNT1 < cycles)
    ;
}

void wait_ms(int wait_time)
{
  int cycles = (int)wait_time * (F_CPU / 1024) / 1000;
  TCNT1 = 0;
  while (TCNT1 < cycles)
    ;
}

///////////////////////////////////
/// 		TIMER FUNCTIONS END 		///
///////////////////////////////////

/////////////////////
/// 		SETUP 		///
/////////////////////

void setup()
{

  DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB0);
  DDRD |= 0xFB;
  DDRC |= (1 << PC4);

  lcd_init(); // Activate LCD

  // Reset timer
  TCCR1A = 0x00;
  TCCR1B = 0x00;

  // Prescaler 1024
  TCCR1B = (1 << CS12) | (1 << CS10);

  // Reset timer counter
  TCNT1 = 0;
}

/////////////////////////
/// 		SETUP END 		///
/////////////////////////

void loop()
{
  /////////////////
  ///		LEDs 		///
  /////////////////

  ADMUX = (1 << REFS0);  // Set the reference voltage to AVCC
  ADMUX |= (0 << MUX0);  // Select A0 as the input channel
  ADCSRA |= (1 << ADEN); // Enable the ADC

  // Conversie A0
  ADCSRA |= (1 << ADSC); // start conversie
  while (ADCSRA & (1 << ADSC))
    ;
  int car_light = ADC;

  // Conversie A1
  ADMUX |= (1 << MUX0);  // select A1 pt ADC
  ADCSRA |= (1 << ADSC); // start conversie
  while (ADCSRA & (1 << ADSC))
    ;
  int outside_light = ADC;

  if (outside_light < 100)
    PORTC |= (1 << PC4);
  else
    PORTC &= ~(1 << PC4);

  if ((PINB & (1 << PB2)))
    if (car_light < 100)
      PORTB |= (1 << PB4);
    else
      PORTB &= ~(1 << PB4);
  else
    PORTB &= ~(1 << PB4);

  ///////////////////////
  /// 		LEDs END 		///
  ///////////////////////

  /////////////////////////////
  ///		DISTANCE SENSOR 		///
  /////////////////////////////
  int treshold_activate = 336;
  int treshold_warning = 200;
  int treshold_emergency = 100;
  bool is_deactivated = false;
  bool is_active = false;
  bool is_warned = false;
  bool is_emergency = false;
  bool distance[16];

  float cm = 0.0181939 * read_ultrasonic_distance(PB0, PB1);

  if ((PIND & (1 << PD3)))
    is_deactivated = false;
  else
    is_deactivated = true;

  if (cm > 336)
    for (int i = 0; i < 16; i++)
      distance[i] = 0;

  if (cm <= 336 && cm > 315)
  {
    for (int i = 0; i < 1; i++)
      distance[i] = 1;
    for (int i = 15; i > 0; i--)
      distance[i] = 0;
  }
  if (cm <= 315 && cm > 294)
  {
    for (int i = 0; i < 2; i++)
      distance[i] = 1;
    for (int i = 15; i > 1; i--)
      distance[i] = 0;
  }
  if (cm <= 294 && cm > 273)
  {
    for (int i = 0; i < 3; i++)
      distance[i] = 1;
    for (int i = 15; i > 2; i--)
      distance[i] = 0;
  };
  if (cm <= 273 && cm > 252)
  {
    for (int i = 0; i < 4; i++)
      distance[i] = 1;
    for (int i = 15; i > 3; i--)
      distance[i] = 0;
  }
  if (cm <= 252 && cm > 231)
  {
    for (int i = 0; i < 5; i++)
      distance[i] = 1;
    for (int i = 15; i > 4; i--)
      distance[i] = 0;
  }
  if (cm <= 231 && cm > 210)
  {
    for (int i = 0; i < 6; i++)
      distance[i] = 1;
    for (int i = 15; i > 5; i--)
      distance[i] = 0;
  }
  if (cm <= 210 && cm > 189)
  {
    for (int i = 0; i < 7; i++)
      distance[i] = 1;
    for (int i = 15; i > 6; i--)
      distance[i] = 0;
  }
  if (cm <= 189 && cm > 168)
  {
    for (int i = 0; i < 8; i++)
      distance[i] = 1;
    for (int i = 15; i > 7; i--)
      distance[i] = 0;
  }
  if (cm <= 168 && cm > 147)
  {
    for (int i = 0; i < 9; i++)
      distance[i] = 1;
    for (int i = 15; i > 8; i--)
      distance[i] = 0;
  }
  if (cm <= 147 && cm > 126)
  {
    for (int i = 0; i < 10; i++)
      distance[i] = 1;
    for (int i = 15; i > 9; i--)
      distance[i] = 0;
  }
  if (cm <= 126 && cm > 105)
  {
    for (int i = 0; i < 11; i++)
      distance[i] = 1;
    for (int i = 15; i > 10; i--)
      distance[i] = 0;
  }
  if (cm <= 105 && cm > 84)
  {
    for (int i = 0; i < 12; i++)
      distance[i] = 1;
    for (int i = 15; i > 11; i--)
      distance[i] = 0;
  }
  if (cm <= 84 && cm > 63)
  {
    for (int i = 0; i < 13; i++)
      distance[i] = 1;
    for (int i = 15; i > 12; i--)
      distance[i] = 0;
  }
  if (cm <= 63 && cm > 42)
  {
    for (int i = 0; i < 14; i++)
      distance[i] = 1;
    for (int i = 15; i > 13; i--)
      distance[i] = 0;
  }
  if (cm <= 42 && cm > 21)
  {
    for (int i = 0; i < 15; i++)
      distance[i] = 1;
    for (int i = 15; i > 14; i--)
      distance[i] = 0;
  }
  if (cm <= 21)
  {
    for (int i = 0; i < 16; i++)
      distance[i] = 1;
  }

  if (!is_deactivated)
  {
    if (cm < treshold_activate)
      is_active = true;
    else
      is_active = false;

    if (cm <= treshold_warning)
      is_warned = true;
    else
      is_warned = false;

    if (cm <= treshold_emergency)
    {
      is_emergency = true;
    }
    else
    {
      is_emergency = false;
    }
  }
  if (is_emergency)
    PORTB |= (1 << PB5);
  else
    PORTB &= ~(1 << PB5);

  ///////////////////////////////////
  /// 		DISTANCE SENSOR END 		///
  ///////////////////////////////////

  ///////////////////
  /// 		LCD 		///
  ///////////////////

  if ((PINB & (1 << PB2)))
    lcd_print_pos(0, 1, "LB");
  else
    lcd_print_pos(0, 1, "  ");

  if ((PINB & (1 << PB3)))
    lcd_print_pos(0, 4, "HB");
  else
    lcd_print_pos(0, 4, "  ");

  if (!is_deactivated)
  {
    if (is_active)
      lcd_print_pos(0, 7, "AC ");
    else
      lcd_print_pos(0, 7, "   ");

    if (is_warned)
      lcd_print_pos(0, 10, "WA ");
    else
      lcd_print_pos(0, 10, "   ");

    if (is_emergency)
      lcd_print_pos(0, 13, "EM ");
    else
      lcd_print_pos(0, 13, "   ");

    for (int i = 0; i < 16; i++)
      if (distance[i])
        lcd_print_pos(1, i, "X");
      else
        lcd_print_pos(1, i, " ");
  }
  else
  {
    lcd_print_pos(0, 7, "SYS DEACT");
    for (int i = 0; i < 16; i++)
      lcd_print_pos(1, i, " ");
  }

  ///////////////////////
  /// 		LCD END 		///
  ///////////////////////
}