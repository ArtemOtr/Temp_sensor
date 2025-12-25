#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define F_CPU 8000000UL
#include <util/delay.h>

#define BAUD_rate 19200
#define UBBR_val (F_CPU/16/BAUD_rate - 1)
#define ADC_VOLT(x) (x * (5.0/1024.0))
#define RX_BUFFER_SIZE 64


volatile static uint64_t mes_delay = 1000; // период измерений
volatile static uint32_t timer_ms_sec = 0; // счетчик милисекунд (обновляемый)
volatile static uint64_t global_timer_ms_sec = 0; // глобальный счетчик
volatile static uint64_t global_time = 0; // глобальное время

volatile static uint8_t global_time_flag = 0; // глобальное время 0 == ничего 1 == delay 2 == глобальный счетчик


#define ARR_SIZE 200
#define GLOBAL_TIME_ARR_SIZE 10

volatile static uint16_t temp_vals[ARR_SIZE];
volatile static uint32_t time_s_vals[ARR_SIZE];
volatile static int16_t global_time_arr[GLOBAL_TIME_ARR_SIZE] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};


volatile static uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile static uint16_t rx_count = 0;	
volatile static uint8_t uart_tx_busy = 1;


void init_arrays(void)
{
    uint16_t i = 0;
    for (i = 0; i < ARR_SIZE; i++) {
        temp_vals[i] = -50;
        time_s_vals[i] = 0;
    }
}

void init_global_time_arr(void){
    uint8_t i;
    for(i = 0; i < GLOBAL_TIME_ARR_SIZE; i++){
        global_time_arr[i] = -1;
    }
}

// *********** TIMER **************

void timer0_init(void){
	TCCR0A |= (0b10 << WGM00); // Clear Timer on Compare Match (CTC) mode (когда достигаем А сбрасываемся в ноль)
	OCR0A = 124; // число для сравнения (после 125 тактов таймер должен сброситься в ноль)
	TIMSK0 |= (1 << OCIE0A); // включаем прерывания после сравнения с A
	TCCR0B |= 0x03; // врубаем предделиттель на 64	

    // по формуле OCR0A = T * F_CPU / prescaler = 1 ms -> прерывание будет раз в милисекунду
}


ISR(TIMER0_COMPA_vect){
	timer_ms_sec++;
    global_timer_ms_sec++;
}


uint8_t check_clock(uint32_t delay_time){ 
    /*чекаем если таймер считает дольше заданного времени
    то сбрасываем и возвращаем единицу
    */ 
    if (delay_time <= timer_ms_sec){
        timer_ms_sec = 0;
        return 1;
    }else{
        return 0;
    }
}

// ****************************

// ***************** ADC *********************

void ADC_init(uint8_t channel)
{
    ADMUX = (1 << REFS0) | (channel & 0x0F);  // установили AVCC это референсное значение для АЦП, выбор канал АЦП              
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) ;  // предделитель на 64 (8MHz / 64 = 125Khz) АЦП работает на 50 - 200
}


uint16_t ADC_conv(void){
    uint8_t adcl = 0;
	uint8_t adch = 0;
    ADCSRA |= (1 << ADSC); // старт преобразвоания
    while (!(ADCSRA & (1 << ADIF)));  

    adcl = ADCL;
	adch = ADCH;

    return (adch << 8 | adcl);

}


// *********** UART **************

void uart_init(void)
{
    UBRR0H = UBBR_val >> 8; // baud rate настраиваем
    UBRR0L = UBBR_val; // baud rate настраиваем
    UCSR0B = (1 << TXEN0) | (1 << RXCIE0) | (1 << RXEN0); // вкл tx, rx и прерывание по rx
    UCSR0B &= ~(1 << UCSZ02); // настройка длины символа на 8 бит
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // настройка длины символа на 8 бит
    DDRD |= (1 << PD1);
}

void uart_tx_char(char data)
{
    while (!(UCSR0A & (1 << UDRE0))); // UDRE0 в UCSR0A показывает, что буффер данных UDR0 пустой
    // в данном случае мы ждем пока в беск цикле пока он освободиться, иначе UDR0 забьет на отпр данные
    UDR0 = data; // записываем char в UDR0
}

// ну здесь все понятно
void uart_tx_string(char *data, uint8_t len)
{   
    uint8_t i;
    for (i = 0; i < len; i++){
        uart_tx_char(data[i]);
    }
}



ISR(INT0_vect){ // прерывание по кнопке
    char data_buffer[48];
    uint16_t i;
    char hello_string[] = "\r\nTemp values since the last button press:\r\n";
    uint8_t len = sizeof(hello_string) - 1;
    uart_tx_string(hello_string, len);

	for (i = 0; i < ARR_SIZE; i++){
        if (temp_vals[i] == -50){
            break;
        }
        uint8_t len = sprintf(data_buffer, "MES TIME: %ld s   TEMP: %d C\r\n", time_s_vals[i], temp_vals[i]); 
        uart_tx_string(data_buffer, len);
    }
    init_arrays();
}

ISR(USART_RX_vect){ // прерывание по получению UART
	
	volatile static uint16_t rx_write_pos = 0;
	
	rx_buffer[rx_write_pos] = UDR0;
	rx_count++; // по сути проверка что есть непрочитанные данные
	rx_write_pos++; // указатель на тек. записываемый байт
	if(rx_write_pos >= RX_BUFFER_SIZE){ // при переполнении буфера, в начало
		rx_write_pos = 0;
	}
	
}

uint8_t uart_read(void){
	static uint16_t rx_read_pos = 0;
	uint8_t data = 0;
	
	data = rx_buffer[rx_read_pos];
	rx_read_pos++;
	rx_count--;
	if(rx_read_pos >= RX_BUFFER_SIZE){
		rx_read_pos = 0;
	}
	return data;
}

uint8_t get_int_from_char(char data){
    return data - '0';
}


void set_global_time_arr(uint8_t *data){

    if (!(data == '\n' || data == '\r' || data == 'g' || data == 'd')){
        uint8_t dig_data = get_int_from_char(data);
        uint8_t i;
        if (global_time_arr[GLOBAL_TIME_ARR_SIZE - 1] != -1){
            init_global_time_arr();
        }
        for (i = 0; i < GLOBAL_TIME_ARR_SIZE; i++) {
            if (global_time_arr[i] == -1){
                global_time_arr[i] = dig_data;
                break;
            }
        }
    }else if(data == 'd'){
        global_time_flag = 1;
    }else if(data == 'g'){
        global_time_flag = 2;
    }
}


ISR(INT1_vect){// прерывание по кнопке
    uint8_t i;
    uint32_t result = 0;
    for (i = 0; i < GLOBAL_TIME_ARR_SIZE; i++) {

        if (global_time_arr[i] == -1)
            break;
        
        result = result * 10 + global_time_arr[i];
    }
    if (global_time_flag == 2){
        global_time = result;
        uint32_t offset = global_time - global_timer_ms_sec/1000;
    
        for (i = 0; i < ARR_SIZE; i++) {
            // time_s_vals[i] = 0;
            if (time_s_vals[i] != 0){
                uint32_t new_time = offset + time_s_vals[i];
                time_s_vals[i] = new_time;
            }
            
        }
        global_timer_ms_sec = global_time * 1000;
    }else if (global_time_flag == 1){
        mes_delay = result * 1000;
    }

    global_time_flag = 0;
    init_global_time_arr();

}


// *************************

int main(void){
    timer0_init();
    uart_init();
    ADC_init(6);
    init_arrays();
    char data_from_uart;

    DDRD &= 0xF3; //0b11110011 INT0 (PD2) и INT1 (PD3) делаем входными
    // DDRD |= 0x08; //0b00001000 настраиваем выход на PD3 для теста
    PORTD |= 0x0C; // включаем подтягивающий на PD2

    EICRA = 0x0A; //0b00001010 настройка прерывания INT0 и INT1 по спаду
    EIMSK = 0x03; //0b00000011 включили прерывания на INT0 и INT1

    sei(); // глобальный вектор прерывания

    while(1){
        if (check_clock(mes_delay)){
            uint16_t raw_adc =  ADC_conv();
            float volt = ADC_VOLT(raw_adc);
            int16_t temp = (volt - 0.5) * 100 + 0.2;
            
            uint16_t i;
            for (i = 0; i < ARR_SIZE; i++){
                if (temp_vals[i] == -50){
                    temp_vals[i] = temp;
                    time_s_vals[i] = global_timer_ms_sec / 1000;
                    break;
                }
             }
           
        }

        if (rx_count > 0){
            data_from_uart = uart_read();
            uart_tx_char(data_from_uart);
            set_global_time_arr(data_from_uart);
            
        }
        
    }
}


