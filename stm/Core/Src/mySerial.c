#include "mySerial.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
//extern int timeout;

extern int rx_receive(uint8_t *out);
extern uint8_t start_char;
extern uint8_t terminating_char;

extern uint8_t* rx_rec;
char str_received[Buffer_Size];
char sent_string[Buffer_Size+2];

extern int input_i[n_ints];
extern float input_f[n_floats];

void empty_string(void){
        for (int i=0;i<Buffer_Size;i++){
            str_received[i]='0';
        }
        for (int i=0;i<Buffer_Size+2;i++){
            sent_string[i]='0';
        }
}

int read_string(void){
        int i=0;
        empty_string();
        do{
            if(rx_receive(rx_rec)){
                str_received[i]=*rx_rec;
                i++;
            }
            if (*rx_rec==terminating_char){
              return 1;
            }
        }while(i<Buffer_Size); // so it checks after 256 for end_char
          return 0; // string limit limit reached
}

void echo_string(void){
  sent_string[0]=start_char;
  for(int i=1;i<Buffer_Size+1;i++){
    sent_string[i]=str_received[i-1];
  }
  sent_string[Buffer_Size+1]=terminating_char;
  CDC_Transmit_FS((uint8_t*)sent_string, Buffer_Size+2);
}
void write_string(void){
  sent_string[0]=start_char;
  for(int i=1;i<Buffer_Size+1;i++){
    sent_string[i]=str_received[i-1];
  }
  sent_string[Buffer_Size+1]=terminating_char;
  CDC_Transmit_FS((uint8_t*)sent_string, Buffer_Size+2);
}

int write_ack(void){
  char acknowledge[2];
  acknowledge[0]=start_char;acknowledge[1]=terminating_char;
  return (CDC_Transmit_FS((uint8_t*)acknowledge, 2));
}

void handle_ints(){
    char buffer[size_int + 1];
    for(int i = 0; i < n_ints; i++){
        for(int j = 0; j < size_int; j++){
            buffer[j] = str_received[i*size_int + j];
        }
        buffer[size_int] = '\0';
        input_i[i] = atoi(buffer);
    }
}
void handle_floats(){
    char buffer[size_float + 1];
    for(int i = 0; i < n_floats; i++){
        for(int j = 0; j < size_float; j++){
            buffer[j] = str_received[i*size_float + j];
        }
        buffer[size_float] = '\0';
        input_f[i] = atof(buffer);
    }
}